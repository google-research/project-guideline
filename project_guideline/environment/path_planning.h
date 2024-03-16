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

#ifndef PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_PATH_PLANNING_H_
#define PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_PATH_PLANNING_H_

#include <cstddef>
#include <deque>
#include <memory>
#include <optional>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "project_guideline/environment/control_signal.h"
#include "project_guideline/environment/obstacle.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/windowed_value_latch.h"

namespace guideline::environment {

inline constexpr float kMAX_RECOMMENDED_DISTANCE_PER_GUIDELINE_SEGMENT = 3;
inline constexpr float kMAX_GUIDELINE_DISTANCE_FOR_ROTATIONAL_MOVEMENT = 15;

// The minimum runner speed required to use the velocity as runner direction,
// otherwise the camera pose is used for direction (e.g. standing still).
inline constexpr float kMinVelocityMetersPerSecond = 1.0;

// The minimum distance from the closest point on guideline to the target
// guidance point.
inline constexpr float kMinTargetPointDistanceMeters = 3.;

// The duration used to project the runner's estimated position based on their
// current velocity and used to adjust the target point distance. For example if
// runner velocity is 3m/s congruent with the line and duration = 2s, the system
// may select a target point an additional 6m ahead.
static const absl::Duration kTargetPointDistanceDuration = absl::Seconds(1);

// Track width to indicate the edges of the track from the closest point on
// guideline.
inline constexpr float kTrackWidthMeters = 3.0;

struct LateralMovementOutput {
  float lateral_movement_meters;
  size_t closest_guideline_point_indx;

  LateralMovementOutput()
      : lateral_movement_meters(0), closest_guideline_point_indx(0) {}
  LateralMovementOutput(float lateral_movement_meters,
                        size_t closest_guideline_point_indx)
      : lateral_movement_meters(lateral_movement_meters),
        closest_guideline_point_indx(closest_guideline_point_indx) {}
};

struct RotationMovementOutput {
  float rotation_movement_degrees;
  std::vector<float> all_rotations;
  size_t num_guideline_points_per_segment;

  RotationMovementOutput()
      : rotation_movement_degrees(0), num_guideline_points_per_segment(0) {}
  RotationMovementOutput(float rotation_movement_degrees,
                         std::vector<float> all_rotations,
                         size_t num_guideline_points_per_segment)
      : rotation_movement_degrees(rotation_movement_degrees),
        all_rotations(all_rotations),
        num_guideline_points_per_segment(num_guideline_points_per_segment) {}
};

struct TurnPointOutput {
  std::optional<::Eigen::Vector3d> turn_point;
  float turn_angle_degrees;
  float turn_point_distance_meters;

  TurnPointOutput()
      : turn_point(std::nullopt),
        turn_angle_degrees(0),
        turn_point_distance_meters(0.) {}
  TurnPointOutput(::Eigen::Vector3d turn_point, float turn_angle_degrees,
                  float turn_point_distance_meters)
      : turn_point(std::make_optional(turn_point)),
        turn_angle_degrees(turn_angle_degrees),
        turn_point_distance_meters(turn_point_distance_meters) {}
  TurnPointOutput(std::optional<::Eigen::Vector3d> turn_point,
                  float turn_angle_degrees, float turn_point_distance_meters)
      : turn_point(turn_point),
        turn_angle_degrees(turn_angle_degrees),
        turn_point_distance_meters(turn_point_distance_meters) {}
};

class ControlSystem {
  // Interface for the control system (control signals extractor).
 public:
  virtual ~ControlSystem() {}
  virtual ControlSignal GenerateControlSignal(
      const util::Transformation& human_position_direction,
      const std::optional<::Eigen::Vector3d>& human_velocity,
      absl::Span<const ::Eigen::Vector3d> guideline_points,
      absl::Span<const Obstacle> obstacles, util::Axis3 vertical_axis) = 0;
  virtual absl::Status ClearEntries(std::optional<size_t> num_entries) = 0;
  virtual size_t NumEntries() = 0;
  virtual ControlSystemOptions GetConfig() const = 0;
};

class SimpleControlSystem : public ControlSystem {
 public:
  static absl::StatusOr<std::unique_ptr<ControlSystem>> Create(
      const ControlSystemOptions& options);
  ControlSignal GenerateControlSignal(
      const util::Transformation& human_position_direction,
      const std::optional<::Eigen::Vector3d>& human_velocity,
      absl::Span<const ::Eigen::Vector3d> guideline_points,
      absl::Span<const Obstacle> obstacles, util::Axis3 vertical_axis) override;
  absl::Status ClearEntries(std::optional<size_t> num_entries) override;
  size_t NumEntries() override;
  ControlSystemOptions GetConfig() const override;

  LateralMovementOutput ComputeLateralMovement(
      const ::Eigen::Vector3d& human_position,
      absl::Span<const ::Eigen::Vector3d> guideline_points,
      util::Axis3 vertical_axis);
  RotationMovementOutput ComputeRotationalMovement(
      const ::Eigen::Vector3d& human_direction,
      absl::Span<const ::Eigen::Vector3d> guideline_points,
      int closest_guideline_point_indx, util::Axis3 vertical_axis);
  RotationMovementOutput ComputeRotationalMovementWithLookAhead(
      const ::Eigen::Vector3d& human_direction,
      absl::Span<const ::Eigen::Vector3d> guideline_points,
      int closest_guideline_point_indx, float rotational_movement_ahead_meter,
      util::Axis3 vertical_axis);
  TurnPointOutput FindTurnPoint(
      const ::Eigen::Vector3d& human_position,
      absl::Span<const ::Eigen::Vector3d> guideline_points,
      int closest_guideline_point_indx, int num_guideline_points_per_segment,
      absl::Span<const float> all_rotations, float immediate_rotation,
      util::Axis3 vertical_axis);

 private:
  explicit SimpleControlSystem(SimpleControlSystemOptions options)
      : options_(options),
        obstacle_latch_(/*value_threshold=*/0.5f,
                        options.obstacle_smoothing_window_frames(),
                        options.obstacle_smoothing_min_interval_frames()) {}
  bool IsStopCondition();
  const ControlSignal PostProcessControlSignals();
  SimpleControlSystemOptions options_;
  absl::Mutex control_signal_history_lock_;
  std::deque<ControlSignal> control_signal_history_
      ABSL_GUARDED_BY(control_signal_history_lock_);
  util::WindowedValueLatch<float> obstacle_latch_;
};

}  // namespace guideline::environment

#endif  // PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_PATH_PLANNING_H_
