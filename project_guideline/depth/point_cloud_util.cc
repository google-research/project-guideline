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

#include "project_guideline/depth/point_cloud_util.h"

#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"

namespace guideline::depth {

using Eigen::Vector3d;

absl::StatusOr<std::vector<Point3D>> ArDepthToPointCloud(
    const util::DepthImageU16& depth_image,
    const util::ConfidenceImageU8& confidence,
    const util::Transformation& world_T_camera,
    const camera::CameraModel& camera_model, const int subsample_step) {
  CHECK_GT(subsample_step, 0);
  CHECK_EQ(
      static_cast<int>(1.0 * depth_image.height() * camera_model.image_width() /
                       camera_model.image_height()),
      depth_image.width())
      << "Image and depth map aspect ratios don't match.";

  std::vector<Point3D> point_cloud;
  // Iterates over pixels, tallying the number of kept points.
  for (int y = 0; y < depth_image.height(); y += subsample_step) {
    for (int x = 0; x < depth_image.width(); x += subsample_step) {
      // Retrieves the depth and confidence values for the pixel at (y, x)
      uint16_t depth_mm = depth_image(y, x);

      constexpr float kMax8BitConfidence = 255.0f;
      const float confidence_normalized = confidence(y, x) / kMax8BitConfidence;

      constexpr double kMmToM = 0.001f;
      const double depth = depth_mm * kMmToM;

      // Depth is valid, converts to 3D point.
      int image_x = static_cast<int>(camera_model.image_width() /
                                     depth_image.width() * x);
      int image_y = static_cast<int>(camera_model.image_height() /
                                     depth_image.height() * y);
      const Eigen::Vector2d pixel(image_x, image_y);
      Vector3d camera_ray_direction;
      if (!camera_model.PixelToRay(pixel, camera_ray_direction)) {
        return absl::InternalError("Unable to project pixel to ray.");
      }

      Vector3d camera_ray = depth * camera_ray_direction;
      // Converts to world coordinate
      Vector3d point = world_T_camera * camera_ray;
      point_cloud.emplace_back(point, confidence_normalized);
    }
  }
  return point_cloud;
}

absl::StatusOr<std::vector<Point3D>> DepthMapToPointCloud(
    const util::DepthImage& depth_map,
    const util::Transformation& world_T_camera,
    const camera::CameraModel& camera_model, const int subsample_step) {
  CHECK_EQ(
      static_cast<int>(1.0 * depth_map.height() * camera_model.image_width() /
                       camera_model.image_height()),
      depth_map.width())
      << "Image and depth map aspect ratios don't match.";

  std::vector<Point3D> point_cloud;
  for (int y = 0; y < depth_map.height(); y += subsample_step) {
    for (int x = 0; x < depth_map.width(); x += subsample_step) {
      const float depth_meters = depth_map(y, x);
      int image_x = static_cast<int>(
          (camera_model.image_width() / depth_map.width()) * x);
      int image_y = static_cast<int>(
          (camera_model.image_height() / depth_map.height()) * y);
      const Eigen::Vector2d pixel(image_x, image_y);

      Vector3d camera_ray;
      if (!camera_model.PixelToRay(pixel, camera_ray)) {
        return absl::InternalError("Unable to project pixel to ray.");
      }

      // We want the point along the ray that has z == depth, so multiply the
      // ray by (depth / ray.z) which will give us the corresponding x/y
      // coordinates.
      Vector3d camera_T_point = (depth_meters / camera_ray.z()) * camera_ray;

      // Converts to world coordinate
      Vector3d world_T_point = world_T_camera * camera_T_point;
      point_cloud.emplace_back(world_T_point, /* confidence= */ 1.0f);
    }
  }
  return point_cloud;
}

}  // namespace guideline::depth
