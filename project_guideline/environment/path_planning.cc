// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "project_guideline/environment/path_planning.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "project_guideline/proto/control_signal.pb.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/geometry.h"

namespace guideline::environment {

using ::Eigen::Vector3d;
using util::Axis3;
using util::Transformation;

namespace {
constexpr int kLogSeconds = 5;

void ComputeTargetPointSignals(
    ControlSignal& control_signal, const SimpleControlSystemOptions& options,
    const Vector3d& human_position, const Vector3d& human_direction,
    const std::optional<Vector3d>& human_velocity,
    const absl::Span<const Vector3d> guideline_points,
    const int closest_guideline_point_indx, const Axis3 vertical_axis) {
  float human_speed = 0.;
  float human_guideline_projection_speed = 0.;
  Vector3d human_velocity_direction = human_direction;
  if (human_velocity.has_value()) {
    auto velocity = *human_velocity;
    ClearAxis(velocity, vertical_axis);

    if (velocity.norm() > kMinVelocityMetersPerSecond) {
      human_velocity_direction = velocity.normalized();
      human_speed = velocity.norm();

      // Project the runner's velocity vector onto the direction of the
      // guideline. If the runner is following the line we can guide them to
      // point further along based on their speed. If the runner is deviating
      // from the line we can choose a closer target point.
      auto closest_point = guideline_points[closest_guideline_point_indx];
      auto p1_indx = std::min(
          closest_guideline_point_indx +
              static_cast<size_t>(kMinTargetPointDistanceMeters *
                                  options.num_guideline_points_per_meter()),
          guideline_points.size() - 1);
      auto p1 = guideline_points[p1_indx];
      auto p1_direction =
          ComputeUnitDirection2D(closest_point, p1, vertical_axis);
      if (std::abs(p1_direction.norm()) >
          std::numeric_limits<double>::epsilon()) {
        human_guideline_projection_speed =
            velocity.dot(p1_direction) / p1_direction.norm();
      }
    }
  }

  float target_distance_meters =
      kMinTargetPointDistanceMeters +
      (std::max(human_guideline_projection_speed, 0.f) *
       absl::ToDoubleSeconds(kTargetPointDistanceDuration));

  size_t target_point_indx = std::min(
      closest_guideline_point_indx +
          static_cast<size_t>(target_distance_meters *
                              options.num_guideline_points_per_meter()),
      guideline_points.size() - 1);
  target_point_indx = std::min(target_point_indx, guideline_points.size() - 1);
  auto target_point = guideline_points[target_point_indx];

  auto target_direction =
      ComputeUnitDirection2D(human_position, target_point, vertical_axis);
  auto target_angle_degrees = util::RadiansToDegrees(util::ComputeAngle2D(
      human_velocity_direction, target_direction, vertical_axis));

  auto closest_guideline_point = guideline_points[closest_guideline_point_indx];
  if (closest_guideline_point_indx < guideline_points.size() - 1) {
    auto next_closest_guideline_point =
        guideline_points[closest_guideline_point_indx + 1];
    auto right_direction = util::ComputePerpendicularDirection2D(
        closest_guideline_point, next_closest_guideline_point, vertical_axis);
    control_signal.right_edge_point =
        closest_guideline_point +
        options.track_width_meters() * right_direction;
    control_signal.left_edge_point =
        closest_guideline_point -
        options.track_width_meters() * right_direction;
  }

  control_signal.speed = human_speed;
  control_signal.velocity_direction = human_velocity_direction;
  control_signal.target_point = target_point;
  control_signal.target_rotation_movement_degrees = -target_angle_degrees;

  control_signal.closest_guideline_point = closest_guideline_point;
  control_signal.guideline_follow_point = target_point;
}
}  // namespace

absl::StatusOr<std::unique_ptr<ControlSystem>> SimpleControlSystem::Create(
    const ControlSystemOptions& options) {
  std::unique_ptr<ControlSystem> base_ptr;
  std::unique_ptr<SimpleControlSystem> derived_ptr;

  auto scs_options = options.simple_control_system_options();
  CHECK_GE(scs_options.distance_per_guideline_segment_meters(), 1);
  if (scs_options.distance_per_guideline_segment_meters() >
      kMAX_RECOMMENDED_DISTANCE_PER_GUIDELINE_SEGMENT) {
    LOG(WARNING) << "Recommended guideline segment size is <= "
                 << kMAX_RECOMMENDED_DISTANCE_PER_GUIDELINE_SEGMENT
                 << " meters.";
  }
  if (scs_options.distance_per_guideline_segment_meters() *
          scs_options.max_num_guideline_segments_to_use() >
      kMAX_GUIDELINE_DISTANCE_FOR_ROTATIONAL_MOVEMENT) {
    LOG_EVERY_N_SEC(WARNING, kLogSeconds)
        << "Recommended guideline distance is <= "
        << kMAX_GUIDELINE_DISTANCE_FOR_ROTATIONAL_MOVEMENT << " meters.";
  }

  derived_ptr = absl::WrapUnique(
      new SimpleControlSystem(options.simple_control_system_options()));
  base_ptr = std::move(derived_ptr);
  return base_ptr;
}

ControlSystemOptions SimpleControlSystem::GetConfig() const {
  auto main_options = ControlSystemOptions();
  auto options = main_options.mutable_simple_control_system_options();
  *options = options_;
  return main_options;
}

size_t SimpleControlSystem::NumEntries() {
  {
    absl::MutexLock lock(&control_signal_history_lock_);
    return control_signal_history_.size();
  }
}

absl::Status SimpleControlSystem::ClearEntries(
    std::optional<size_t> num_entries) {
  {
    absl::MutexLock lock(&control_signal_history_lock_);
    if (num_entries.has_value()) {
      size_t num_entries_value = num_entries.value();
      if (control_signal_history_.size() < num_entries_value) {
        return absl::FailedPreconditionError(
            "ArcoreTrackingBasedHuman::ClearEntries: More entries to clear "
            "than "
            "available in the control_signal_history.");
      }
      for (size_t i = 0; i < num_entries_value; ++i) {
        control_signal_history_.pop_front();
      }
    } else {
      // If nullopt then clear all entries.
      control_signal_history_.clear();
    }
  }
  return absl::OkStatus();
}

// Returns immediate lateral movement required for human to be close to the
// guideline. Also returns the index of the closest guideline point to the
// human's position.
LateralMovementOutput SimpleControlSystem::ComputeLateralMovement(
    const Vector3d& human_position, absl::Span<const Vector3d> guideline_points,
    const Axis3 vertical_axis) {
  std::vector<float> distances;
  auto num_guideline_points = guideline_points.size();
  distances.reserve(num_guideline_points);
  for (auto& guideline_point : guideline_points) {
    distances.push_back(util::ComputeDistance2D(human_position, guideline_point,
                                                vertical_axis));
  }
  auto closest_guideline_point_indx =
      std::min_element(distances.begin(), distances.end()) - distances.begin();
  auto closest_guideline_point =
      guideline_points.at(closest_guideline_point_indx);
  auto intermediate_guideline_point_indx =
      std::min(static_cast<size_t>(
                   closest_guideline_point_indx +
                   options_.num_guideline_points_per_meter() *
                       options_.intermediate_guideline_point_distance_meters()),
               num_guideline_points - 1);
  auto intermediate_guideline_point =
      guideline_points.at(intermediate_guideline_point_indx);

  // Compute human's laternal movement by projecting human's current position
  // onto the line formed by closest and intermediate guideline points.
  double distance = util::ComputePerpendicularDistanceFromLine2D(
      human_position, closest_guideline_point, intermediate_guideline_point,
      vertical_axis);
  return LateralMovementOutput(distance, closest_guideline_point_indx);
}

// Returns immediate rotational movement required to align human with the
// immediate direction of the guideline. Additionally also returns angle
// between human's direction and all guideline segments. This output is
// required for computing the turn point and angle. This is done to avoid
// recalculation of angles when computing the turn point and angle. Lastly
// return number of guideline points in `guideline_points` covering a single
// segment. This helps to find the guideline_point where the turn begins.
RotationMovementOutput SimpleControlSystem::ComputeRotationalMovement(
    const Vector3d& human_direction,
    absl::Span<const Vector3d> guideline_points,
    int closest_guideline_point_indx, const Axis3 vertical_axis) {
  auto num_guideline_points = guideline_points.size();
  auto indices_covered_by_all_segments =
      static_cast<size_t>(options_.distance_per_guideline_segment_meters() *
                          options_.max_num_guideline_segments_to_use() *
                          options_.num_guideline_points_per_meter());
  auto farthest_guideline_point_indx =
      std::min(closest_guideline_point_indx + indices_covered_by_all_segments,
               num_guideline_points - 1);
  auto num_guideline_points_per_segment = std::max(
      static_cast<size_t>(1),
      static_cast<size_t>(options_.num_guideline_points_per_meter() *
                          options_.distance_per_guideline_segment_meters()));
  std::vector<size_t> indices_to_use;
  for (size_t i = closest_guideline_point_indx; i < num_guideline_points;
       i += num_guideline_points_per_segment) {
    indices_to_use.push_back(i);
  }
  if (indices_to_use.size() < 2) {
    // Not enough information available to compute the rotational movement.
    LOG_EVERY_N_SEC(WARNING, kLogSeconds)
        << "Not enough guideline points available to compute rotational "
           "movement.";
    return RotationMovementOutput(0, std::vector<float>(), 0);
  }

  std::vector<float> all_rotations;
  std::vector<float> rotations;
  all_rotations.reserve(indices_to_use.size());
  rotations.reserve(indices_to_use.size());
  for (size_t i = 1; i < indices_to_use.size(); ++i) {
    auto dst_indx = indices_to_use[i];
    auto src_indx = indices_to_use[i - 1];
    auto segment_direction = util::ComputeUnitDirection2D(
        guideline_points.at(src_indx), guideline_points.at(dst_indx),
        vertical_axis);
    auto angle = util::RadiansToDegrees(util::ComputeAngle2D(
        human_direction, segment_direction, vertical_axis));
    all_rotations.push_back(angle);
    if (dst_indx <= farthest_guideline_point_indx) {
      rotations.push_back(angle);
    }
  }

  float rotation_movement_degrees =
      std::accumulate(rotations.begin(), rotations.end(), 0.0) /
      rotations.size();

  return RotationMovementOutput(rotation_movement_degrees, all_rotations,
                                num_guideline_points_per_segment);
}
RotationMovementOutput
SimpleControlSystem::ComputeRotationalMovementWithLookAhead(
    const Vector3d& human_direction,
    absl::Span<const Vector3d> guideline_points,
    int closest_guideline_point_indx, float rotational_movement_ahead_meter,
    const Axis3 vertical_axis) {
  auto indices_skip =
      static_cast<size_t>(rotational_movement_ahead_meter *
                          options_.num_guideline_points_per_meter());
  auto closest_guideline_point_indx_offset =
      closest_guideline_point_indx + indices_skip;
  return ComputeRotationalMovement(human_direction, guideline_points,
                                   closest_guideline_point_indx_offset,
                                   vertical_axis);
}

TurnPointOutput SimpleControlSystem::FindTurnPoint(
    const Vector3d& human_position, absl::Span<const Vector3d> guideline_points,
    int closest_guideline_point_indx, int num_guideline_points_per_segment,
    absl::Span<const float> all_rotations, float immediate_rotation,
    const Axis3 vertical_axis) {
  if (all_rotations.size() < 2) {
    return TurnPointOutput();
  }
  std::vector<float> diff_angles;
  diff_angles.reserve(all_rotations.size());
  std::adjacent_difference(all_rotations.begin(), all_rotations.end(),
                           std::back_inserter(diff_angles));
  auto max_angle_diff_indx =
      std::max_element(diff_angles.begin(), diff_angles.end()) -
      diff_angles.begin();
  auto turn_angle = all_rotations.at(max_angle_diff_indx) - immediate_rotation;
  if (std::abs(turn_angle) < options_.turn_angle_threshold_degrees()) {
    return TurnPointOutput();
  }

  auto turn_point_indx = closest_guideline_point_indx +
                         max_angle_diff_indx * num_guideline_points_per_segment;
  auto turn_point = guideline_points.at(turn_point_indx);

  float turn_point_distance_meters =
      util::ComputeDistance2D(human_position, turn_point, vertical_axis);
  // If turn point is too close the to human don't provide turn point control.
  if (turn_point_distance_meters < options_.min_turn_point_distance_meters()) {
    return TurnPointOutput();
  }
  return TurnPointOutput(turn_point, turn_angle, turn_point_distance_meters);
}

StopReason SimpleControlSystem::IsStopCondition() {
  {
    absl::MutexLock lock(&control_signal_history_lock_);

    auto current_control_signal = control_signal_history_.back();
    // Out of guideline scenario.
    if (std::isnan(current_control_signal.lateral_movement_meters)) {
      return StopReason::STOP_REASON_LATERAL_NAN;
    }

    // Deviation from the guideline scenario.
    size_t num_entries = control_signal_history_.size();
    size_t lateral_stop_min_consecutive_frames =
        static_cast<size_t>(options_.lateral_stop_min_consecutive_frames());
    auto lateral_movement_abs_max_threshold_meters =
        options_.lateral_movement_abs_max_threshold_meters();
    if (std::abs(current_control_signal.lateral_movement_meters) >=
            lateral_movement_abs_max_threshold_meters &&
        num_entries >= lateral_stop_min_consecutive_frames) {
      bool has_in_range_movement = false;
      // If for `min_num_consecutive_frames` the immediate lateral adjustment is
      // >= `lateral_movement_abs_max_threshold_meters` then we trigger the stop
      // signal.
      for (int i = static_cast<int>(num_entries - 1);
           i >=
           static_cast<int>(num_entries - lateral_stop_min_consecutive_frames);
           --i) {
        auto prior_signal = control_signal_history_.at(i);
        if (prior_signal.stop) {
          break;
        } else if (std::abs(prior_signal.lateral_movement_meters) <
                   lateral_movement_abs_max_threshold_meters) {
          has_in_range_movement = true;
          break;
        }
      }
      if (!has_in_range_movement) {
        return StopReason::STOP_REASON_NO_IN_RANGE_MOVEMENT;
      }
    }
    return StopReason::STOP_REASON_UNSPECIFIED;
  }
}

// Post processes and returns final control signal to be communicated to the
// human.
const ControlSignal SimpleControlSystem::PostProcessControlSignals() {
  StopReason stop_reason = IsStopCondition();
  if (stop_reason != StopReason::STOP_REASON_UNSPECIFIED) {
    ControlSignal stop_signal;
    stop_signal.stop = true;
    stop_signal.stop_reason = stop_reason;
    obstacle_latch_.Reset();
    return stop_signal;
  }
  ControlSignal updated_control_signal;
  {
    absl::MutexLock lock(&control_signal_history_lock_);
    updated_control_signal = control_signal_history_.back();
  }
  float final_lateral_movement_meters =
      updated_control_signal.lateral_movement_meters;
  float final_rotational_movement_degrees =
      updated_control_signal.rotation_movement_degrees;

  // Take the direction of the human into consideration and adjust the lateral
  // movement. If the human's direction is perpendicular to the guideline then
  // lateral adjustment doesn't accomplish much as in the resultant position
  // the runner will be still same number of meters away from the guideline. And
  // when the human is parallel to the guideline we need full lateral
  // adjustment. Overall, following logic captures the interaction between
  // lateral and rotational movements required of the human.
  final_lateral_movement_meters =
      final_lateral_movement_meters * std::abs(std::cos(util::DegreesToRadians(
                                          final_rotational_movement_degrees)));

  // No significant lateral adjustment required.
  if (std::abs(final_lateral_movement_meters) <=
      options_.lateral_movement_abs_min_threshold_meters()) {
    final_lateral_movement_meters = 0;
  }

  // No significant rotational adjustment required.
  if (std::abs(final_rotational_movement_degrees) <=
      options_.rotational_movement_abs_min_threshold_degrees()) {
    final_rotational_movement_degrees = 0;
  }

  // If large immediate adjustments are required then ignore turn signal.
  float final_turn_angle_degrees = updated_control_signal.turn_angle_degrees;
  float final_turn_point_distance_meters =
      updated_control_signal.turn_point_distance_meters;
  auto final_turn_point = updated_control_signal.turn_point;
  if (std::abs(final_lateral_movement_meters) >=
          options_.lateral_movement_abs_max_threshold_meters() ||
      std::abs(final_rotational_movement_degrees) >=
          options_.rotational_movement_abs_max_threshold_degrees()) {
    // Ignore turn signal as rotational movement is large.
    final_turn_angle_degrees = 0;
    final_turn_point_distance_meters = 0;
    final_turn_point = std::nullopt;
  }

  updated_control_signal.lateral_movement_meters =
      final_lateral_movement_meters;
  updated_control_signal.rotation_movement_degrees =
      final_rotational_movement_degrees;
  updated_control_signal.turn_point = final_turn_point;
  updated_control_signal.turn_angle_degrees = final_turn_angle_degrees;
  updated_control_signal.turn_point_distance_meters =
      final_turn_point_distance_meters;

  return updated_control_signal;
}

ControlSignal SimpleControlSystem::GenerateControlSignal(
    const Transformation& human_position_direction,
    const std::optional<Vector3d>& human_velocity,
    absl::Span<const Vector3d> guideline_points,
    absl::Span<const Obstacle> obstacles, const Axis3 vertical_axis) {
  const Vector3d& human_position = human_position_direction.p();
  const Vector3d& human_direction =
      util::ComputeForwardDirectionFromQuaternion(human_position_direction.q());

  ControlSignal current_control_signal;

  if (guideline_points.empty()) {
    // If no guideline points are provided then enter obstacle-only mode.
    current_control_signal.obstacle_only_mode = true;
  } else {
    auto lateral_movement =
        ComputeLateralMovement(human_position, guideline_points, vertical_axis);
    auto rotational_movement = ComputeRotationalMovement(
        human_direction, guideline_points,
        lateral_movement.closest_guideline_point_indx, vertical_axis);
    auto rotational_movement_ahead = ComputeRotationalMovementWithLookAhead(
        human_direction, guideline_points,
        lateral_movement.closest_guideline_point_indx,
        options_.rotational_movement_ahead_meters(), vertical_axis);
    auto turn_point = FindTurnPoint(
        human_position, guideline_points,
        lateral_movement.closest_guideline_point_indx,
        rotational_movement.num_guideline_points_per_segment,
        rotational_movement.all_rotations,
        rotational_movement.rotation_movement_degrees, vertical_axis);

    current_control_signal.lateral_movement_meters =
        lateral_movement.lateral_movement_meters;
    current_control_signal.rotation_movement_degrees =
        rotational_movement.rotation_movement_degrees;
    current_control_signal.rotation_movement_ahead_degrees =
        rotational_movement_ahead.rotation_movement_degrees;
    current_control_signal.turn_point = turn_point.turn_point;
    current_control_signal.turn_angle_degrees = turn_point.turn_angle_degrees;
    current_control_signal.turn_point_distance_meters =
        turn_point.turn_point_distance_meters;

    ComputeTargetPointSignals(current_control_signal, options_, human_position,
                              human_direction, human_velocity, guideline_points,
                              lateral_movement.closest_guideline_point_indx,
                              vertical_axis);
  }

  current_control_signal.obstacle_ahead =
      obstacle_latch_.Update(obstacles.empty() ? 0.0f : 1.0f);

  {
    absl::MutexLock lock(&control_signal_history_lock_);
    if (control_signal_history_.size() >= options_.max_history_to_keep()) {
      control_signal_history_.pop_front();
    }
    control_signal_history_.push_back(current_control_signal);
  }
  return PostProcessControlSignals();
}

}  // namespace guideline::environment
