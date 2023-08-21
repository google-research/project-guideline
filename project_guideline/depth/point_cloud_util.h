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

#ifndef PROJECT_GUIDELINE_DEPTH_POINT_CLOUD_UTIL_H_
#define PROJECT_GUIDELINE_DEPTH_POINT_CLOUD_UTIL_H_

#include <vector>

#include "absl/status/statusor.h"
#include "Eigen/Core"
#include "project_guideline/camera/camera_model.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/transformation.h"

namespace guideline::depth {

struct Point3D {
  Point3D(const Eigen::Vector3d coordinate, const double confidence)
      : coordinate(coordinate), confidence(confidence) {}

  const Eigen::Vector3d coordinate;
  const double confidence;
};

// Creates a point cloud from the given ARCore depth image.
absl::StatusOr<std::vector<Point3D>> ArDepthToPointCloud(
    const util::DepthImageU16& depth_image,
    const util::ConfidenceImageU8& confidence,
    const util::Transformation& world_T_camera,
    const camera::CameraModel& camera_model, int subsample_step = 1);

// Creates a point cloud from the given depth map.
// The depth_map values are in meters.
absl::StatusOr<std::vector<Point3D>> DepthMapToPointCloud(
    const util::DepthImage& depth_map,
    const util::Transformation& world_T_camera,
    const camera::CameraModel& camera_model, int subsample_step = 1);

}  // namespace guideline::depth

#endif  // PROJECT_GUIDELINE_DEPTH_POINT_CLOUD_UTIL_H_
