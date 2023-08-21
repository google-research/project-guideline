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

#include "project_guideline/environment/guideline_aggregator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iterator>
#include <memory>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "Eigen/Core"  // keep include
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/cubic_spline.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/hit_test_util.h"
#include "project_guideline/util/math.h"

namespace guideline {
namespace environment {

using ::Eigen::MatrixXd;
using ::Eigen::Vector3d;
using ::Eigen::VectorXd;
using environment_internal::FindClosestPoint;
using environment_internal::PolyEval;
using util::Axis3;
using util::CubicSpline;
using util::HitResult;

namespace {
inline constexpr int kLogSeconds = 5;
}  // namespace

namespace environment_internal {
float PolyEval(const VectorXd& coeffs, const float x) {
  float p = 0;
  for (auto iter = coeffs.end() - 1; iter >= coeffs.begin(); --iter) {
    p = *iter + p * x;
  }
  return p;
}

std::pair<size_t, double> FindClosestPoint(const std::vector<Vector3d>& points,
                                           const int start_index,
                                           const Vector3d& target_point) {
  size_t min_index = start_index;
  double min_distance = (target_point - points[start_index]).norm();
  for (int i = start_index + 1; i < points.size(); ++i) {
    auto& point = points[i];
    double distance = (target_point - point).norm();
    if (distance < min_distance) {
      min_index = i;
      min_distance = distance;
    }
  }
  return {min_index, min_distance};
}
}  // namespace environment_internal

bool GuidelineBoxPoint::IsKeypointInBox(const HitResult& hit_result) const {
  return ((min_x <= hit_result.hit_pose.p().x() &&
           hit_result.hit_pose.p().x() <= max_x) &&
          (min_y <= hit_result.hit_pose.p().y() &&
           hit_result.hit_pose.p().y() <= max_y) &&
          (min_z <= hit_result.hit_pose.p().z() &&
           hit_result.hit_pose.p().z() <= max_z));
}

bool GuidelineBoxPoint::AddKeypointToBox(const HitResult& hit_result,
                                         int64_t timestamp) {
  if (IsKeypointInBox(hit_result)) {
    timestamp_keypoints[timestamp].push_back(hit_result);
    return true;
  } else {
    return false;
  }
}

void GuidelineBoxPoint::ClearEntries(
    std::optional<absl::Span<const int64_t>> timestamps) {
  if (timestamps.has_value()) {
    for (float timestamp : timestamps.value()) {
      timestamp_keypoints.erase(timestamp);
    }
  } else {
    timestamp_keypoints.clear();
  }
}

std::pair<float, float> GuidelineBoxPoint::CountAndConfidence() const {
  if (timestamp_keypoints.empty()) {
    return std::make_pair(0, 0);
  }

  float sum_confidence = 0;
  float count = 0;
  for (const auto& kv : timestamp_keypoints) {
    for (const auto& hit_result : kv.second) {
      sum_confidence += hit_result.confidence;
      count += 1;
    }
  }
  return std::make_pair(count, sum_confidence / count);
}

absl::Status SortedGuidelineBoxPointAggregator::ComputeGuideline() {
  std::vector<Vector3d> points;
  std::vector<float> distances;
  {
    absl::MutexLock lock(&sorted_box_points_lock_);
    if (sorted_box_points_.size() < 2) {
      LOG_EVERY_N_SEC(WARNING, kLogSeconds)
          << "SortedGuidelineBoxPointAggregator::ComputeGuideline: Not enough "
             "box "
             "points.";
      return absl::FailedPreconditionError(
          "SortedGuidelineBoxPointAggregator::ComputeGuideline: Not enough "
          "box points.");
    }
    points.reserve(sorted_box_points_.size());
    distances.reserve(sorted_box_points_.size());
    for (auto& box_point : sorted_box_points_) {
      auto [count, confidence] = box_point.CountAndConfidence();
      if ((count >= options_.num_points_threshold())) {
        points.push_back(box_point.center);
        distances.push_back(box_point.distance_to_anchor);
      }
    }
  }

  if (distances.size() < 2) {
    LOG_EVERY_N_SEC(WARNING, kLogSeconds)
        << "SortedGuidelineBoxPointAggregator::GetGuideline: Not enough high "
           "confidence box points.";
    return absl::FailedPreconditionError(
        "SortedGuidelineBoxPointAggregator::GetGuideline: Not enough high "
        "confidence box points.");
  }

  // Generate inputs for fitting.
  // Inputs are normalized distance in range (0,1).
  float distance_covered_by_guideline = distances.back();
  CHECK_GT(distance_covered_by_guideline, 0);
  std::transform(distances.begin(), distances.end(), distances.begin(),
                 [distance_covered_by_guideline](float value) -> float {
                   return value / distance_covered_by_guideline;
                 });

  // Find data points corresponding to every `num_meters_per_spline_`.
  // We will fit spline to these points.
  float normalized_distance_covered_by_a_spline =
      distance_covered_by_guideline / options_.num_meters_per_spline();
  std::vector<float> normalized_distance_per_spline;
  normalized_distance_per_spline.reserve(
      std::max(2, static_cast<int>(normalized_distance_covered_by_a_spline)));
  for (float i = 0; i <= 1.0;
       i += 1.0 / normalized_distance_covered_by_a_spline) {
    normalized_distance_per_spline.push_back(i);
  }
  if (normalized_distance_per_spline.back() < 1.0) {
    normalized_distance_per_spline.push_back(1);
  }

  // Spline fitting. Input is distance and output is coordinate.
  CubicSpline spline_x(CubicSpline::kNatural);
  CubicSpline spline_y(CubicSpline::kNatural);
  CubicSpline spline_z(CubicSpline::kNatural);

  int previous_indx = -1;
  for (auto normalized_distance : normalized_distance_per_spline) {
    // Find closest corresponding distance and use its index.
    auto lower = std::lower_bound(distances.begin(), distances.end(),
                                  normalized_distance);
    auto indx = std::distance(distances.begin(), lower);
    // Following logic ensure we get strictly monotonic distances which is a
    // necessary condition for spline fitting.
    if (indx > previous_indx) {
      Vector3d point = points.at(indx);
      float distance = distances.at(indx);
      CHECK_EQ(CubicSpline::kNoError, spline_x.AddPoint(distance, point.x()));
      CHECK_EQ(CubicSpline::kNoError, spline_y.AddPoint(distance, point.y()));
      CHECK_EQ(CubicSpline::kNoError, spline_z.AddPoint(distance, point.z()));
      previous_indx = indx;
    }
  }

  CubicSpline::SplineError x_fit_error = spline_x.Fit();
  CubicSpline::SplineError y_fit_error = spline_y.Fit();
  CubicSpline::SplineError z_fit_error = spline_z.Fit();

  if (x_fit_error != CubicSpline::kNoError ||
      y_fit_error != CubicSpline::kNoError ||
      z_fit_error != CubicSpline::kNoError) {
    LOG_EVERY_N_SEC(WARNING, kLogSeconds)
        << "SortedGuidelineBoxPointAggregator::ComputeGuideline: Spline "
           "fit failed: "
        << x_fit_error << ":" << y_fit_error << ":" << z_fit_error;
    return absl::FailedPreconditionError(
        "SortedGuidelineBoxPointAggregator::ComputeGuideline: Spline fitting "
        "failed.");
  }

  int num_fitted_guideline_points =
      static_cast<int>(distance_covered_by_guideline *
                           options_.num_guideline_points_per_meter() +
                       options_.forward_extrapolation_meters() *
                           options_.num_guideline_points_per_meter() +
                       options_.backward_extrapolation_meters() *
                           options_.num_guideline_points_per_meter());

  std::vector<Vector3d> new_guideline_points;
  new_guideline_points.reserve(num_fitted_guideline_points);
  double start_value =
      -options_.backward_extrapolation_meters() / distance_covered_by_guideline;
  double end_value = 1 + options_.forward_extrapolation_meters() /
                             distance_covered_by_guideline;
  double delta = (end_value - start_value) / num_fitted_guideline_points;
  CHECK_GT(delta, 0);
  while (start_value <= end_value) {
    float x, y, z;
    if (spline_x.Interpolate(start_value, &x) == CubicSpline::kNoError &&
        spline_y.Interpolate(start_value, &y) == CubicSpline::kNoError &&
        spline_z.Interpolate(start_value, &z) == CubicSpline::kNoError) {
      new_guideline_points.push_back(Vector3d(x, y, z));
    }
    start_value += delta;
  }

  {
    absl::MutexLock lock(&guideline_points_lock_);
    guideline_points_ = new_guideline_points;
  }
  return absl::OkStatus();
}

void SortedGuidelineBoxPointAggregator::AggregateKeypoints(
    absl::Span<const HitResult> hit_results, int64_t timestamp,
    Axis3 vertical_axis) {
  if (hit_results.empty()) {
    LOG_EVERY_N_SEC(WARNING, kLogSeconds)
        << "SortedGuidelineBoxPointAggregator::AggregateKeypoints: Empty "
           "`hit_results`.";
    return;
  }

  if (timestamps_.size() >= options_.max_history_to_keep()) {
    CHECK_OK(ClearEntries(options_.max_history_to_keep() / 2));
  }

  {
    absl::MutexLock lock(&sorted_box_points_lock_);
    if (sorted_box_points_.empty()) {
      // Create anchor point.
      sorted_box_points_.push_back(GuidelineBoxPoint(
          options_.box_size_meters(), timestamp, hit_results.at(0),
          /*distance_to_anchor*/ 0));
    }

    bool hit_result_added;
    std::vector<float> distance_to_box_points;
    distance_to_box_points.reserve(sorted_box_points_.size());
    const GuidelineBoxPoint* closest_box_point_ptr = nullptr;
    float distance_to_anchor;
    for (const auto& hit_result : hit_results) {
      hit_result_added = false;
      distance_to_box_points.clear();
      for (auto& box_point : sorted_box_points_) {
        if (box_point.AddKeypointToBox(hit_result, timestamp)) {
          hit_result_added = true;
          closest_box_point_ptr = &box_point;
          break;
        } else {
          distance_to_box_points.push_back(util::ComputeDistance2D(
              hit_result.hit_pose.p(), box_point.center, vertical_axis));
        }
      }
      if (!hit_result_added) {
        if (closest_box_point_ptr == nullptr) {
          auto closest_guideline_point_indx =
              std::min_element(distance_to_box_points.begin(),
                               distance_to_box_points.end()) -
              distance_to_box_points.begin();
          closest_box_point_ptr =
              &(sorted_box_points_.at(closest_guideline_point_indx));
        }
        // Create new box point.
        distance_to_anchor = closest_box_point_ptr->distance_to_anchor +
                             util::ComputeDistance2D(
                                 hit_result.hit_pose.p(),
                                 closest_box_point_ptr->center, vertical_axis);
        auto new_box_point =
            GuidelineBoxPoint(options_.box_size_meters(), timestamp, hit_result,
                              distance_to_anchor);
        // Find appropriater index to insert the new box point.
        // std::deque<GuidelineBoxPoint>::iterator it =
        // sorted_box_points_.begin();
        auto it = std::find_if(
            sorted_box_points_.begin(), sorted_box_points_.end(),
            [distance_to_anchor](auto box_point) {
              return (box_point.distance_to_anchor > distance_to_anchor);
            });
        sorted_box_points_.insert(it, new_box_point);
      }
    }
    timestamps_.push(timestamp);
  }
  auto status = ComputeGuideline();
  if (!status.ok()) {
    LOG_EVERY_N_SEC(WARNING, kLogSeconds) << status.message();
  }
}

// This api is not available through the virtual base class
// `GuidelineAggregator`. We only use it for unit testing.
const std::deque<GuidelineBoxPoint>&
SortedGuidelineBoxPointAggregator::GetSortedBoxPoints() const {
  return sorted_box_points_;
}

absl::StatusOr<std::vector<Vector3d>>
SortedGuidelineBoxPointAggregator::GetGuideline() {
  {
    absl::MutexLock lock(&guideline_points_lock_);
    if (guideline_points_.empty()) {
      return absl::FailedPreconditionError(
          "SortedGuidelineBoxPointAggregator::GetGuideline: Not guideline "
          "points "
          "detected.");
    }
    return guideline_points_;
  }
}

absl::Status SortedGuidelineBoxPointAggregator::ClearEntries(
    std::optional<size_t> num_entries) {
  {
    absl::MutexLock lock(&sorted_box_points_lock_);
    if (!num_entries.has_value()) {
      while (!timestamps_.empty()) {
        timestamps_.pop();
      }
      sorted_box_points_.clear();

      absl::MutexLock lock(&guideline_points_lock_);
      guideline_points_.clear();

      return absl::OkStatus();
    }
    size_t num_entries_value = num_entries.value();

    // Find timestamps to clear.
    if (num_entries_value > timestamps_.size()) {
      return absl::InvalidArgumentError(
          "SortedGuidelineBoxPointAggregator::ClearEntries: More entries to "
          "clear than available.");
    }

    std::vector<int64_t> timestamps_to_clear;
    timestamps_to_clear.reserve(num_entries_value);
    for (int i = 0; i < num_entries_value; ++i) {
      timestamps_to_clear.push_back(timestamps_.front());
      timestamps_.pop();
    }

    // Find and remove box points which don't have any hit results after
    // clearing the entries corresponding to `timestamps`.
    auto itr = sorted_box_points_.begin();
    while (itr != sorted_box_points_.end()) {
      itr->ClearEntries(timestamps_to_clear);
      if (itr->CountAndConfidence().first == 0) {
        itr = sorted_box_points_.erase(itr);
      } else {
        ++itr;
      }
    }

    // Adjust `distance_to_anchor` for the remaining box points.
    itr = sorted_box_points_.begin();
    float distance_to_subtract = itr->distance_to_anchor;
    if (distance_to_subtract != 0) {
      while (itr != sorted_box_points_.end()) {
        itr->distance_to_anchor -= distance_to_subtract;
        ++itr;
      }
    }
  }

  if (!ComputeGuideline().ok()) {
    // Clear existing guideline as it function of the cleared entries.
    absl::MutexLock lock(&guideline_points_lock_);
    guideline_points_.clear();
  }

  ComputeGuideline().IgnoreError();
  return absl::OkStatus();
}

absl::StatusOr<std::unique_ptr<GuidelineAggregator>>
SortedGuidelineBoxPointAggregator::Create(
    const GuidelineAggregatorOptions& options) {
  CHECK_GT(options.sorted_guideline_box_point_aggregator_options()
               .max_history_to_keep(),
           1);
  std::unique_ptr<GuidelineAggregator> base_ptr;
  std::unique_ptr<SortedGuidelineBoxPointAggregator> derived_ptr;
  derived_ptr = absl::WrapUnique(new SortedGuidelineBoxPointAggregator(
      options.sorted_guideline_box_point_aggregator_options()));
  base_ptr = std::move(derived_ptr);
  return base_ptr;
}

GuidelineAggregatorOptions SortedGuidelineBoxPointAggregator::GetConfig()
    const {
  auto main_options = GuidelineAggregatorOptions();
  auto options =
      main_options.mutable_sorted_guideline_box_point_aggregator_options();
  *options = options_;
  return main_options;
}

absl::StatusOr<std::unique_ptr<GuidelineAggregator>>
LocalTemporalRegressionBasedGuidelineAggregator::Create(
    const GuidelineAggregatorOptions& options) {
  CHECK_GT(
      options.local_temporal_regression_based_guideline_aggregator_options()
          .max_history_to_keep(),
      1);
  return std::unique_ptr<GuidelineAggregator>(
      absl::WrapUnique(new LocalTemporalRegressionBasedGuidelineAggregator(
          options
              .local_temporal_regression_based_guideline_aggregator_options())));  // NOLINT(whitespace/line_length)
}

absl::StatusOr<FitCurveOutput>
LocalTemporalRegressionBasedGuidelineAggregator::FitCurve(
    std::vector<double> x, std::vector<double> y1, std::vector<double> y2,
    int degree_to_use, bool first_interval, bool last_interval) {
  // `degree_to_use` + 1: 1 for the intercept.
  MatrixXd m_x = MatrixXd::Zero(x.size(), degree_to_use + 1);
  VectorXd v_y1 = VectorXd::Map(&y1.front(), y1.size());
  VectorXd v_y2 = VectorXd::Map(&y2.front(), y2.size());

  for (int i = 0; i < x.size(); i++) {
    for (int j = 0; j <= degree_to_use; j++) {
      m_x(i, j) = std::pow(x[i], j);
    }
  }

  VectorXd coeff_y1 = m_x.householderQr().solve(v_y1);
  VectorXd coeff_y2 = m_x.householderQr().solve(v_y2);

  std::vector<double> new_x;
  std::vector<double> fitted_y1;
  std::vector<double> fitted_y2;

  float start_x = x.front();
  float stop_x = x.back();
  double direction_x = 1.0;
  if (x.back() < x.front()) {
    direction_x = -1;
  }
  if (options_.use_fitted_curve_for_extrapolation()) {
    if (first_interval) {
      // In the first interval we do backward extrapolation.
      start_x -= direction_x * options_.backward_extrapolation_meters();
    }
    if (last_interval) {
      stop_x += direction_x * options_.forward_extrapolation_meters();
    }
  }

  const int num_points =
      static_cast<int>(std::abs(stop_x - start_x) *
                       options_.num_guideline_points_per_meter()) +
      1;
  new_x.reserve(num_points);
  fitted_y1.reserve(num_points);
  fitted_y2.reserve(num_points);

  const double increment_x =
      direction_x / options_.num_guideline_points_per_meter();

  float nx = start_x;
  for (int i = 0; i < num_points; i++, nx += increment_x) {
    float y1_eval = PolyEval(coeff_y1, nx);
    float y2_eval = PolyEval(coeff_y2, nx);
    if (!std::isinf(y1_eval) && !std::isnan(y1_eval) && !std::isinf(y2_eval) &&
        !std::isinf(y2_eval)) {
      new_x.push_back(nx);
      fitted_y1.push_back(y1_eval);
      fitted_y2.push_back(y2_eval);
    }
  }
  return FitCurveOutput{.independent_variable_values = new_x,
                        .dependent_variable1_values = fitted_y1,
                        .dependent_variable2_values = fitted_y2};
}

std::deque<std::vector<Vector3d>>
LocalTemporalRegressionBasedGuidelineAggregator::
    TemporalLocalFittingWithExogeneousVariableSelection() {
  {
    absl::MutexLock lock(&guideline_point_cloud_lock_);
    size_t num_timestamps = guideline_point_cloud_.size();
    if (num_timestamps == 0) {
      return std::deque<std::vector<Vector3d>>{};
    }
    float num_intervals_float =
        (static_cast<float>(num_timestamps) /
         static_cast<float>(options_.time_interval_size() -
                            options_.time_interval_overlap()));
    size_t num_intervals =
        static_cast<size_t>(std::max(std::round(num_intervals_float), 1.0f));

    std::deque<std::vector<Vector3d>> time_interval_based_fitted_points;
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    std::vector<double> fitted_xs;
    std::vector<double> fitted_ys;
    std::vector<double> fitted_zs;
    bool first_interval = false;
    bool last_interval = false;

    for (size_t interval = 0; interval < num_intervals; ++interval) {
      xs.clear();
      ys.clear();
      zs.clear();
      fitted_xs.clear();
      fitted_ys.clear();
      fitted_zs.clear();
      first_interval = false;
      last_interval = false;

      if (interval == 0) {
        first_interval = true;
      }
      if (interval == num_intervals - 1) {
        last_interval = true;
      }

      size_t start_indx = interval * (options_.time_interval_size() -
                                      options_.time_interval_overlap());
      size_t end_indx =
          std::min(start_indx + options_.time_interval_size(), num_timestamps);

      // If last interval then make `end_indx` `num_timestamps`.
      // If `num_intervals_float` is round to upper integer than we fit linear
      // curve to avoid overfitting. E.g. if `num_intervals_float` is 3.7
      // it will be rounded to 4, in this case we do linear fitting.
      // If `num_intervals_float` is 4.4 it will be rounded to 4, in this case
      // we use the curve degree by the config as in this case the timestamp
      // interval size is bigger and has more data.
      int degree_to_use = options_.fitted_curve_degree();

      for (size_t i = start_indx; i < end_indx; ++i) {
        for (const HitResult& point :
             std::get<1>(guideline_point_cloud_.at(i))) {
          xs.push_back(point.hit_pose.p().x());
          ys.push_back(point.hit_pose.p().y());
          zs.push_back(point.hit_pose.p().z());
        }
      }
      auto exogeneous_axis = Axis3::kX;
      double dx = util::StandardDeviation(xs);
      double dy = util::StandardDeviation(ys);
      double dz = util::StandardDeviation(zs);

      // Using `>=` to avoid the situation where `dx=dy=dz`.
      if (dx >= dy && dx >= dz) {
        exogeneous_axis = Axis3::kX;
        auto fitted_values =
            FitCurve(xs, ys, zs, degree_to_use, first_interval, last_interval);
        CHECK_OK(fitted_values);
        fitted_xs = fitted_values->independent_variable_values;
        fitted_ys = fitted_values->dependent_variable1_values;
        fitted_zs = fitted_values->dependent_variable2_values;
      } else if (dy >= dx && dy >= dz) {
        exogeneous_axis = Axis3::kY;
        auto fitted_values =
            FitCurve(ys, xs, zs, degree_to_use, first_interval, last_interval);
        CHECK_OK(fitted_values);
        fitted_ys = fitted_values->independent_variable_values;
        fitted_xs = fitted_values->dependent_variable1_values;
        fitted_zs = fitted_values->dependent_variable2_values;
      } else if (dz >= dx && dz >= dy) {
        exogeneous_axis = Axis3::kZ;
        auto fitted_values =
            FitCurve(zs, xs, ys, degree_to_use, first_interval, last_interval);
        CHECK_OK(fitted_values);
        fitted_zs = fitted_values->independent_variable_values;
        fitted_xs = fitted_values->dependent_variable1_values;
        fitted_ys = fitted_values->dependent_variable2_values;
      }
      if (vertical_axis_.has_value() &&
          vertical_axis_.value() == exogeneous_axis) {
        LOG_EVERY_N_SEC(WARNING, kLogSeconds) << "Exogeneous axis"
                                              << " == vertical_axis:";
      }
      std::vector<Vector3d> fitted_points;
      for (size_t i = 0; i < fitted_xs.size(); ++i) {
        fitted_points.push_back(
            Vector3d(fitted_xs.at(i), fitted_ys.at(i), fitted_zs.at(i)));
      }
      time_interval_based_fitted_points.push_back(fitted_points);
    }
    return time_interval_based_fitted_points;
  }
}

absl::Status
LocalTemporalRegressionBasedGuidelineAggregator::ComputeGuideline() {
  // Piecewise curve approximation.
  const std::deque<std::vector<Vector3d>>& time_interval_based_fitted_points =
      TemporalLocalFittingWithExogeneousVariableSelection();
  if (time_interval_based_fitted_points.empty()) {
    return absl::FailedPreconditionError(
        "No piecewise curve approximation available.");
  }

  // Spatio-temporal stitching.
  int last_merged_indx = 0;
  int stitch_point_index_fitted_points = -1;
  int stitch_point_index_control_points = -1;
  bool all_points_stitched = true;
  std::vector<Vector3d> control_points;
  for (int i = 0; i < time_interval_based_fitted_points.size(); ++i) {
    const std::vector<Vector3d>& fit_result =
        time_interval_based_fitted_points.at(i);
    if (i == 0) {
      control_points = fit_result;
      continue;
    }

    stitch_point_index_fitted_points = -1;
    stitch_point_index_control_points = -1;
    all_points_stitched = true;
    for (int j = 0; j < fit_result.size(); ++j) {
      Vector3d point = fit_result[j];
      auto closest_control_point =
          FindClosestPoint(control_points, last_merged_indx, point);
      if (closest_control_point.second <
          options_.max_distance_for_closest_point_meters()) {
        stitch_point_index_fitted_points = j;
        stitch_point_index_control_points = closest_control_point.first;
        // Update the closest point in `control_points` such that it is mean
        // of its control and current points.
        control_points[stitch_point_index_control_points] =
            (control_points[stitch_point_index_control_points] + point) / 2.;
      } else {
        all_points_stitched = false;
      }
    }
    // If all points stitched do nothing.
    if (all_points_stitched) continue;
    if (stitch_point_index_control_points != -1) {
      // Remove points after the stitch point from the control points.
      if (stitch_point_index_control_points < control_points.size() - 1) {
        control_points = std::vector(
            control_points.begin(),
            control_points.begin() + stitch_point_index_control_points + 1);
      }
      last_merged_indx = stitch_point_index_control_points;
      // Add incoming new points after the stitch point to the control points.
      control_points.insert(
          control_points.end(),
          fit_result.begin() + stitch_point_index_fitted_points + 1,
          fit_result.end());
    } else {
      // No stitch point detected.
      last_merged_indx = control_points.size() - 1;
      // Add all incoming points to the control points.
      control_points.insert(control_points.end(), fit_result.begin(),
                            fit_result.end());
    }
  }

  // Spline fitting equivalent.
  if (control_points.size() < 2) {
    absl::MutexLock lock(&guideline_points_lock_);
    guideline_points_ = control_points;
    return absl::OkStatus();
  }

  std::vector<double> distances;
  distances.reserve(control_points.size());
  distances.push_back(0);
  Vector3d point;
  Vector3d previous_point;
  double distance_from_anchor = 0;
  for (int i = 1; i < control_points.size(); ++i) {
    point = control_points[i];
    previous_point = control_points[i - 1];
    double d = (point - previous_point).norm();
    distance_from_anchor += d;
    distances.push_back(distance_from_anchor);
  }

  // Inputs for spline fitting are normalized distance in range (0,1).
  double distance_covered_by_guideline = distances.back();
  for (auto iter = distances.begin(); iter < distances.end(); ++iter) {
    *iter = *iter / distance_covered_by_guideline;
  }

  // Range for evaluation based on the extrapolation parameters.
  double eval_start =
      -options_.backward_extrapolation_meters() / distance_covered_by_guideline;
  double eval_end = 1 + options_.forward_extrapolation_meters() /
                            distance_covered_by_guideline;
  int num_fitted_points =
      static_cast<int>((distance_covered_by_guideline +
                        options_.backward_extrapolation_meters() +
                        options_.forward_extrapolation_meters()) *
                       options_.num_guideline_points_per_meter());

  if (options_.use_fitted_curve_for_extrapolation()) {
    // If `use_fitted_curve_for_extrapolation` is enabled then no need to do
    // extrapolation in the fitted spline.
    eval_start = 0.0;
    eval_end = 1.0;
    num_fitted_points =
        static_cast<int>(distance_covered_by_guideline *
                         options_.num_guideline_points_per_meter());
  }

  double distance_increment =
      (eval_end - eval_start) / static_cast<double>(num_fitted_points);
  std::vector<Vector3d> fitted_points;
  fitted_points.reserve(num_fitted_points);
  int upper_index = 0;
  double distance = eval_start;
  Vector3d p1, p2;
  double d1, d2;
  for (int i = 0; i < num_fitted_points; ++i) {
    auto it = std::upper_bound(distances.begin() + upper_index, distances.end(),
                               distance);
    if (it == distances.end()) {
      upper_index = distances.size() - 1;
    } else {
      upper_index = std::distance(distances.begin(), it);
    }

    if (upper_index == 0) {
      p1 = control_points[0];
      d1 = distances[0];
      p2 = control_points[1];
      d2 = distances[1];
    } else {
      if (upper_index - 1 < 0 || upper_index - 1 > control_points.size() - 1) {
        continue;
      }
      p1 = control_points[upper_index - 1];
      d1 = distances[upper_index - 1];
      p2 = control_points[upper_index];
      d2 = distances[upper_index];
    }

    // Linear interpolation between the points.
    fitted_points.push_back(p1 + ((distance - d1) / (d2 - d1)) * (p2 - p1));

    distance += distance_increment;
  }

  absl::MutexLock lock(&guideline_points_lock_);
  guideline_points_ = fitted_points;
  return absl::OkStatus();
}

absl::Status LocalTemporalRegressionBasedGuidelineAggregator::ClearEntries(
    std::optional<size_t> num_entries) {
  {
    absl::MutexLock lock(&guideline_point_cloud_lock_);
    size_t num_entries_value;
    if (num_entries.has_value()) {
      num_entries_value = num_entries.value();
    } else {
      num_entries_value = guideline_point_cloud_.size();
    }

    if (num_entries_value > guideline_point_cloud_.size()) {
      return absl::FailedPreconditionError(
          "LocalTemporalRegressionBasedGuidelineAggregator::ClearEntries: More "
          "entries to clear than "
          "available in the guideline_point_cloud.");
    }

    for (size_t i = 0; i < num_entries_value; ++i) {
      guideline_point_cloud_.pop_front();
    }
  }

  if (!ComputeGuideline().ok()) {
    // Clear existing guideline as it function of the cleared entries.
    absl::MutexLock lock(&guideline_points_lock_);
    guideline_points_.clear();
  }

  ComputeGuideline().IgnoreError();
  return absl::OkStatus();
}

void LocalTemporalRegressionBasedGuidelineAggregator::AggregateKeypoints(
    absl::Span<const HitResult> hit_results, int64_t timestamp,
    Axis3 vertical_axis) {
  if (hit_results.empty()) {
    LOG_EVERY_N_SEC(WARNING, kLogSeconds)
        << "LocalTemporalRegressionBasedGuidelineAggregator::"
           "AggregateKeypoints: Empty "
           "`hit_results`.";
    return;
  }
  bool clear_history = false;
  {
    absl::MutexLock lock(&guideline_point_cloud_lock_);
    if (guideline_point_cloud_.size() >= options_.max_history_to_keep()) {
      clear_history = true;
    }
  }
  if (clear_history) {
    // Clear half the history.
    CHECK_OK(ClearEntries(options_.max_history_to_keep() / 2));
  }

  {
    absl::MutexLock lock(&guideline_point_cloud_lock_);
    std::vector<HitResult> new_hit_results;
    for (const auto& hit_result : hit_results) {
      new_hit_results.push_back(hit_result);
    }
    guideline_point_cloud_.push_back(
        std::make_tuple(timestamp, new_hit_results));
    vertical_axis_ = vertical_axis;
  }
  ComputeGuideline().IgnoreError();
}

absl::StatusOr<std::vector<Vector3d>>
LocalTemporalRegressionBasedGuidelineAggregator::GetGuideline() {
  {
    absl::MutexLock lock(&guideline_points_lock_);
    if (guideline_points_.empty()) {
      return absl::FailedPreconditionError(
          "LocalTemporalRegressionBasedGuidelineAggregator::GetGuideline: No "
          "guideline "
          "points "
          "detected.");
    }
    return guideline_points_;
  }
}

GuidelineAggregatorOptions
LocalTemporalRegressionBasedGuidelineAggregator::GetConfig() const {
  GuidelineAggregatorOptions main_options;
  *main_options
       .mutable_local_temporal_regression_based_guideline_aggregator_options() =
      options_;
  return main_options;
}

absl::StatusOr<std::unique_ptr<GuidelineAggregator>> GetGuidelineAggregator(
    const GuidelineAggregatorOptions& options) {
  switch (options.guideline_aggregator_options_case()) {
    case GuidelineAggregatorOptions::
        kSortedGuidelineBoxPointAggregatorOptions: {
      return SortedGuidelineBoxPointAggregator::Create(options);
    }
    case (GuidelineAggregatorOptions::
              kLocalTemporalRegressionBasedGuidelineAggregatorOptions): {
      return LocalTemporalRegressionBasedGuidelineAggregator::Create(options);
    }
    case GuidelineAggregatorOptions::GUIDELINE_AGGREGATOR_OPTIONS_NOT_SET: {
      return absl::InvalidArgumentError("Invalid guideline aggregator.");
    }
  }
}

}  // namespace environment
}  // namespace guideline
