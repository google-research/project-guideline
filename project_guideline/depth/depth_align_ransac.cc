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

#include "project_guideline/depth/depth_align_ransac.h"

#include <limits>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include "absl/status/status.h"
#include "absl/strings/str_format.h"

namespace guideline::depth {

namespace {

using motion::TrackingFeature;

constexpr uint32_t kMinInliers = 10;

constexpr double kCvRansacReprojThreshold = 0.01;
constexpr size_t kCvRansacMaxIters = 500;
constexpr double kCvRansacConfidence = 0.99;
constexpr size_t kCvRansacRefineIters = 10;

constexpr double kMaxTrackingFeatureDepthMeters = 40.0;
constexpr double kInvalidDepth = -1;

bool ValidTrackingFeature(double z) {
  return z > std::numeric_limits<double>::epsilon() &&
         z < kMaxTrackingFeatureDepthMeters;
}

bool ValidTrackingFeature(const TrackingFeature& feature) {
  return ValidTrackingFeature(feature.camera_t_feature.z());
}

inline double DepthMapValueAtFeature(const TrackingFeature& feature,
                                     const camera::CameraModel& camera_model,
                                     const Eigen::Vector2d& depth_map_scale,
                                     const util::DepthImage& depth_map) {
  Eigen::Vector2d pixel;
  if (camera_model.PointToPixel(feature.camera_t_feature.cast<double>(),
                                pixel)) {
    int x = pixel.x() * depth_map_scale.x();
    int y = pixel.y() * depth_map_scale.y();

    if (x >= 0 && x < depth_map.width() && y >= 0 && y < depth_map.height()) {
      return depth_map(y, x);
    }
  }

  return kInvalidDepth;
}

absl::StatusOr<DepthAlignParams> PerformRANSAC(
    const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst) {
  if (src.size() < kMinInliers) {
    return absl::InvalidArgumentError("Not enough valid features");
  }

  // Estimate 2D affine transformation using OpenCV RANSAC.
  std::vector<uchar> inlier_mask;
  cv::Mat affine = cv::estimateAffinePartial2D(
      src, dst, inlier_mask, cv::RANSAC, kCvRansacReprojThreshold,
      kCvRansacMaxIters, kCvRansacConfidence, kCvRansacRefineIters);

  if (affine.empty()) {
    // Likely due to 0 inliers.
    return absl::InternalError("Failed to estimate affine transformation");
  }

  size_t num_inliers = cv::countNonZero(inlier_mask);
  if (num_inliers < kMinInliers) {
    return absl::InternalError(absl::StrFormat(
        "Not enough inliers in RANSAC: %d < %d", num_inliers, kMinInliers));
  }

  double scale = affine.at<double>(0, 0);
  double shift = affine.at<double>(1, 2);

  return DepthAlignParams{scale, shift};
}

}  // namespace

absl::StatusOr<DepthAlignParams> DepthAlignRansac(
    const util::DepthImage& depth_map,
    const std::vector<TrackingFeature>& tracking_features,
    const camera::CameraModel& camera_model) {
  if (tracking_features.size() < kMinInliers) {
    return absl::InvalidArgumentError("Not enough features");
  }

  Eigen::Vector2d depth_map_scale = {
      1.0 * depth_map.width() / camera_model.image_width(),
      1.0 * depth_map.height() / camera_model.image_height()};

  std::vector<cv::Point2f> src;
  std::vector<cv::Point2f> dst;

  for (const auto& feature : tracking_features) {
    if (!ValidTrackingFeature(feature)) {
      continue;
    }
    const double depth = DepthMapValueAtFeature(feature, camera_model,
                                                depth_map_scale, depth_map);
    if (depth < std::numeric_limits<double>::epsilon()) {
      continue;
    }

    const double inv_depth = 1.0 / depth;
    const double inv_z_feature = 1.0 / feature.camera_t_feature.z();

    src.emplace_back(0, inv_depth);
    dst.emplace_back(0, inv_z_feature);
  }

  return PerformRANSAC(src, dst);
}

absl::StatusOr<DepthAlignParams> DepthAlignRansacImageNormalized(
    const cv::Mat_<float>& depth_map, bool depth_map_is_inverse_depth,
    const std::vector<Eigen::Vector3f>& image_normalized_tracking_features) {
  if (image_normalized_tracking_features.size() < kMinInliers) {
    return absl::InvalidArgumentError("Not enough features");
  }

  std::vector<cv::Point2f> src;
  std::vector<cv::Point2f> dst;

  for (const auto& feature : image_normalized_tracking_features) {
    if (feature.x() >= 0 && feature.x() < depth_map.cols && feature.y() >= 0 &&
        feature.y() < depth_map.rows) {
      const double depth_val = depth_map(feature.y(), feature.x());
      const double inv_depth =
          depth_map_is_inverse_depth ? depth_val : 1.0 / depth_val;
      const double inv_z_feature = 1.0 / feature.z();
      src.emplace_back(0, inv_depth);
      dst.emplace_back(0, inv_z_feature);
    }
  }

  return PerformRANSAC(src, dst);
}

}  // namespace guideline::depth
