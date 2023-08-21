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

#ifndef PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_POINT_CLOUD_H_
#define PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_POINT_CLOUD_H_

#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "project_guideline/camera/camera_model.h"
#include "project_guideline/depth/point_cloud_util.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/transformation.h"

namespace guideline::environment {

// Point cloud is a list of 3D points in the world coordinate system.
// More details can be found in doc go/guideline-obstacle-detection-v0
// The current occupancy map is frame-centric stateless. In the future versions,
// We will extend it to be stateful.
class PointCloud {
 public:
  static std::unique_ptr<PointCloud> Create(const PointCloudOptions& options);
  absl::StatusOr<std::vector<depth::Point3D>> GetPointCloud();

  // Update the list of 3D points using the latest depth map from ARCore.
  void UpdatePointCloud(const util::DepthImageU16& depth,
                        const util::ConfidenceImageU8& confidence,
                        const util::Transformation& world_T_camera,
                        const camera::CameraModel& camera_model);

  // Update the list of 3D points using depth estimation map.
  // The depth_map values are in meters.
  void UpdatePointCloud(const util::DepthImage& depth_map,
                        const util::Transformation& world_T_camera,
                        const camera::CameraModel& camera_model);

 private:
  explicit PointCloud(const FrameBasedPointCloudOptions options)
      : options_(options) {}
  absl::Mutex point_cloud_lock_;
  std::vector<depth::Point3D> point_cloud_ ABSL_GUARDED_BY(point_cloud_lock_);
  const FrameBasedPointCloudOptions options_;
};

}  // namespace guideline::environment

#endif  // PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_POINT_CLOUD_H_
