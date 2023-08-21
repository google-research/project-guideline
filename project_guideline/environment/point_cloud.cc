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

#include "project_guideline/environment/point_cloud.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/synchronization/mutex.h"
#include "project_guideline/util/image.h"

namespace guideline::environment {
namespace {
using depth::ArDepthToPointCloud;
using depth::DepthMapToPointCloud;
using depth::Point3D;
}  // namespace

std::unique_ptr<PointCloud> PointCloud::Create(
    const PointCloudOptions& options) {
  CHECK_GT(options.frame_based_point_cloud_options().subsample_step(), 0)
      << "sample interval must be >= 0.";
  return absl::WrapUnique(
      new PointCloud(options.frame_based_point_cloud_options()));
}

absl::StatusOr<std::vector<Point3D>> PointCloud::GetPointCloud() {
  absl::MutexLock lock(&point_cloud_lock_);
  if (point_cloud_.empty()) {
    return absl::FailedPreconditionError(
        "PointCloud::GetPointCloud: No point cloud found.");
  }
  return point_cloud_;
}

void PointCloud::UpdatePointCloud(const util::DepthImage& depth_map,
                                  const util::Transformation& world_T_camera,
                                  const camera::CameraModel& camera_model) {
  auto point_cloud = DepthMapToPointCloud(
      depth_map, world_T_camera, camera_model, options_.subsample_step());

  if (!point_cloud.ok()) {
    LOG(WARNING) << "Failed to convert depth map to point cloud.";
    return;
  }

  absl::MutexLock lock(&point_cloud_lock_);
  point_cloud_ = std::move(*point_cloud);
}

void PointCloud::UpdatePointCloud(const util::DepthImageU16& depth,
                                  const util::ConfidenceImageU8& confidence,
                                  const util::Transformation& world_T_camera,
                                  const camera::CameraModel& camera_model) {
  auto point_cloud =
      ArDepthToPointCloud(depth, confidence, world_T_camera, camera_model,
                          options_.subsample_step());
  if (!point_cloud.ok()) {
    LOG(WARNING) << "Failed to convert depth map to point cloud.";
  }
  absl::MutexLock lock(&point_cloud_lock_);
  point_cloud_ = std::move(*point_cloud);
}

}  // namespace guideline::environment
