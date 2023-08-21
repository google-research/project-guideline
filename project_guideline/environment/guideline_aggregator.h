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

#ifndef PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_GUIDELINE_AGGREGATOR_H_
#define PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_GUIDELINE_AGGREGATOR_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/hit_test_util.h"

namespace guideline::environment {

namespace environment_internal {
// Evaluates a polynomial defined by the `coeffs` vector for the given value of
// `x`. First entry of `coeffs` is the intercept followed by the coefficients
// of the order/degree in ascending order.
float PolyEval(const ::Eigen::VectorXd& coeffs, float x);

// Returns the index to the closest 3D point given a set of points and a
// starting index to start the search from. Uses Euclidean distance as a
// measure of closeness.
std::pair<size_t, double> FindClosestPoint(
    const std::vector<::Eigen::Vector3d>& points, int start_index,
    const ::Eigen::Vector3d& target_point);
}  // namespace environment_internal

struct GuidelineBoxPoint {
  // Not making this const as in future versions this could be adaptive.
  ::Eigen::Vector3d center;
  absl::flat_hash_map<int64_t, std::vector<util::HitResult>>
      timestamp_keypoints;
  float box_size;
  float min_x, min_y, min_z;
  float max_x, max_y, max_z;
  float distance_to_anchor;  // Measuring unit is meters.

  GuidelineBoxPoint(const ::Eigen::Vector3d& center, float box_size)
      : center(center),
        box_size(box_size),
        min_x(center.x() - box_size / 2),
        min_y(center.y() - box_size / 2),
        min_z(center.z() - box_size / 2),
        max_x(center.x() + box_size / 2),
        max_y(center.y() + box_size / 2),
        max_z(center.z() + box_size / 2),
        distance_to_anchor(0) {}

  // Constructor to create box_point based on the first entry.
  GuidelineBoxPoint(float box_size, int64_t timestamp,
                    const util::HitResult& hit_result, float distance_to_anchor)
      : center(hit_result.hit_pose.p()),
        box_size(box_size),
        min_x(center.x() - box_size / 2),
        min_y(center.y() - box_size / 2),
        min_z(center.z() - box_size / 2),
        max_x(center.x() + box_size / 2),
        max_y(center.y() + box_size / 2),
        max_z(center.z() + box_size / 2),
        distance_to_anchor(distance_to_anchor) {
    timestamp_keypoints[timestamp].push_back(hit_result);
  }

  bool IsKeypointInBox(const util::HitResult& hit_result) const;
  bool AddKeypointToBox(const util::HitResult& hit_result, int64_t timestamp);
  void ClearEntries(std::optional<absl::Span<const int64_t>> timestamps);
  std::pair<float, float> CountAndConfidence() const;
};

struct FitCurveOutput {
  std::vector<double> independent_variable_values;
  std::vector<double> dependent_variable1_values;
  std::vector<double> dependent_variable2_values;
};

// Guideline representation is simply a list of Coordinate3D indicating the
// progression of the guideline from origin/anchor.

class GuidelineAggregator {
  // Interface for Guideline aggregator.
 public:
  virtual ~GuidelineAggregator() {}
  virtual GuidelineAggregatorOptions GetConfig() const = 0;
  virtual void AggregateKeypoints(absl::Span<const util::HitResult> hit_results,
                                  int64_t timestamp,
                                  util::Axis3 vertical_axis) = 0;
  // Returns sequence of Coordinate3D indicating the progression of the
  // guideline. The first entry in this sequence is the origin of the guideline
  // and the last entry is the farthest point in the guideline, respectively.
  // The progression is in the direction the runner is running i.e. the runner
  // is running towards the end of this sequence.
  virtual absl::StatusOr<std::vector<::Eigen::Vector3d>> GetGuideline() = 0;
  virtual absl::Status ClearEntries(std::optional<size_t> num_entries) = 0;
};

// Maintains a distance based sorted list of guideline box points and fits
// linear spline to get the guideline points to be used in downstream tasks.
class SortedGuidelineBoxPointAggregator : public GuidelineAggregator {
 public:
  static absl::StatusOr<std::unique_ptr<GuidelineAggregator>> Create(
      const GuidelineAggregatorOptions& options);
  void AggregateKeypoints(absl::Span<const util::HitResult> hit_results,
                          int64_t timestamp,
                          util::Axis3 vertical_axis) override;
  const std::deque<GuidelineBoxPoint>& GetSortedBoxPoints() const;
  absl::StatusOr<std::vector<::Eigen::Vector3d>> GetGuideline() override;
  absl::Status ClearEntries(std::optional<size_t> num_entries) override;
  GuidelineAggregatorOptions GetConfig() const override;

 private:
  explicit SortedGuidelineBoxPointAggregator(
      const SortedGuidelineBoxPointAggregatorOptions options)
      : options_(options) {}
  absl::Status ComputeGuideline();

  absl::Mutex sorted_box_points_lock_;
  std::deque<GuidelineBoxPoint> sorted_box_points_
      ABSL_GUARDED_BY(sorted_box_points_lock_);
  std::queue<int64_t> timestamps_;
  absl::Mutex guideline_points_lock_;
  std::vector<::Eigen::Vector3d> guideline_points_
      ABSL_GUARDED_BY(guideline_points_lock_);
  const SortedGuidelineBoxPointAggregatorOptions options_;
};

// Maintains a distance based sorted list of guideline box points and fits
// linear spline to get the guideline points to be used in downstream tasks.
class LocalTemporalRegressionBasedGuidelineAggregator
    : public GuidelineAggregator {
 public:
  static absl::StatusOr<std::unique_ptr<GuidelineAggregator>> Create(
      const GuidelineAggregatorOptions& options);
  void AggregateKeypoints(absl::Span<const util::HitResult> hit_results,
                          int64_t timestamp,
                          util::Axis3 vertical_axis) override;
  absl::StatusOr<std::vector<::Eigen::Vector3d>> GetGuideline() override;
  absl::Status ClearEntries(std::optional<size_t> num_entries) override;
  GuidelineAggregatorOptions GetConfig() const override;

  // Following functions are made public to facilitate testing.
  absl::StatusOr<FitCurveOutput> FitCurve(std::vector<double> x,
                                          std::vector<double> y1,
                                          std::vector<double> y2,
                                          int degree_to_use,
                                          // If true then the input is from the
                                          // first temporal window.
                                          bool first_interval = false,
                                          // If true then the input is from the
                                          // last temporal window.
                                          bool last_interval = false);

 private:
  explicit LocalTemporalRegressionBasedGuidelineAggregator(
      const LocalTemporalRegressionBasedGuidelineAggregatorOptions options)
      : options_(options) {}
  absl::Status ComputeGuideline();
  std::deque<std::vector<::Eigen::Vector3d>>
  TemporalLocalFittingWithExogeneousVariableSelection();

  absl::Mutex guideline_point_cloud_lock_;
  std::deque<std::tuple<int64_t, std::vector<util::HitResult>>>
      guideline_point_cloud_ ABSL_GUARDED_BY(guideline_point_cloud_lock_);
  std::queue<int64_t> timestamps_;
  absl::Mutex guideline_points_lock_;
  std::vector<::Eigen::Vector3d> guideline_points_
      ABSL_GUARDED_BY(guideline_points_lock_);
  const LocalTemporalRegressionBasedGuidelineAggregatorOptions options_;
  std::optional<util::Axis3> vertical_axis_;
};

absl::StatusOr<std::unique_ptr<GuidelineAggregator>> GetGuidelineAggregator(
    const GuidelineAggregatorOptions& options);

}  // namespace guideline::environment

#endif  // PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_GUIDELINE_AGGREGATOR_H_
