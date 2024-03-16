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

// The GuidanceSystem combines results from motion tracking (camera pose) and
// line detection (keypoints from segmentation mask) and builds a stateful
// representation of the world. This information is then used to generate
// guidance signals to the user.

#ifndef PROJECT_GUIDELINE_VISION_GUIDANCE_SYSTEM_H_
#define PROJECT_GUIDELINE_VISION_GUIDANCE_SYSTEM_H_

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "project_guideline/camera/camera_model.h"
#include "project_guideline/environment/control_signal.h"
#include "project_guideline/environment/environment.h"
#include "project_guideline/environment/path_planning.h"
#include "project_guideline/logging/guideline_logger.h"
#include "project_guideline/motion/tracking_feature.h"
#include "project_guideline/proto/control_signal.pb.h"
#include "project_guideline/util/hit_test_util.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/transformation.h"

namespace guideline::environment {

using ControlSignalCallback =
    std::function<void(const ControlSignal& control_signal)>;

struct PendingFeatures {
  PendingFeatures() = default;
  PendingFeatures(int64_t timestamp_us,
                  const util::Transformation& world_t_camera,
                  const std::vector<motion::TrackingFeature>& features)
      : timestamp_us(timestamp_us),
        world_t_camera(world_t_camera),
        features(features) {}
  int64_t timestamp_us = 0;
  util::Transformation world_t_camera;
  std::vector<motion::TrackingFeature> features;
};

class GuidanceSystem {
 public:
  static absl::StatusOr<std::unique_ptr<GuidanceSystem>> Create(
      const GuidanceSystemOptions& options,
      std::shared_ptr<logging::GuidelineLogger> logger,
      std::unique_ptr<Environment> environment,
      std::unique_ptr<ControlSystem> control_system);

  void OnCameraPose(int64_t timestamp_us,
                    const util::Transformation& world_t_camera,
                    std::shared_ptr<camera::CameraModel> camera_model);
  void OnDetection(int64_t timestamp_us,
                   const std::vector<Eigen::Vector3f>& keypoints,
                   std::shared_ptr<const util::ConfidenceMask> guideline_mask,
                   std::shared_ptr<const util::DepthImage> depth_map);
  void OnTrackingStateChanged(bool is_tracking);
  void OnArDepth(int64_t timestamp_us, const util::DepthImageU16& depth_image,
                 const util::ConfidenceImageU8& confidence);
  void OnTrackingFeatures(int64_t timestamp_us,
                          const std::vector<motion::TrackingFeature>& features);

  void AddControlSignalCallback(
      const ControlSignalCallback& control_signal_callback)
      ABSL_LOCKS_EXCLUDED(control_signal_callbacks_mutex_);

  absl::StatusOr<std::vector<Eigen::Vector3d>> GetGuideline();
  absl::StatusOr<std::vector<Eigen::Vector2d>> GetObstacles();
  absl::StatusOr<util::Transformation> CurrentPositionAndDirection();

  absl::Status Stop();

 private:
  explicit GuidanceSystem(const GuidanceSystemOptions& options,
                          std::shared_ptr<logging::GuidelineLogger> logger,
                          std::unique_ptr<Environment> environment,
                          std::unique_ptr<ControlSystem> control_system);

  void UpdateStatefulRepresentation(
      const util::Transformation& world_t_camera,
      absl::Span<const util::HitResult> ordered_hit_results,
      int64_t timestamp_us);

  void ProcessDepthMap(int64_t timestamp_us,
                       const util::Transformation& world_t_camera,
                       const util::DepthImage& depth_map,
                       const PendingFeatures& features);

  void ResetAndSendStopSignal(StopReason reason);

  std::shared_ptr<camera::CameraModel> GetCameraModel()
      ABSL_LOCKS_EXCLUDED(camera_model_mutex_);

  const GuidanceSystemOptions options_;
  std::shared_ptr<logging::GuidelineLogger> logger_;
  std::unique_ptr<Environment> environment_;
  std::unique_ptr<ControlSystem> control_system_;

  absl::Mutex camera_model_mutex_;
  std::shared_ptr<camera::CameraModel> camera_model_
      ABSL_GUARDED_BY(camera_model_mutex_) = nullptr;

  absl::Mutex pending_camera_poses_mutex_;
  std::deque<std::pair<const int64_t, util::Transformation>>
      pending_detection_poses_ ABSL_GUARDED_BY(pending_camera_poses_mutex_);
  std::deque<std::pair<const int64_t, util::Transformation>>
      pending_feature_poses_ ABSL_GUARDED_BY(pending_camera_poses_mutex_);
  std::deque<std::pair<const int64_t, PendingFeatures>>
      pending_tracking_features_ ABSL_GUARDED_BY(pending_camera_poses_mutex_);

  absl::Mutex control_signal_callbacks_mutex_;
  std::vector<ControlSignalCallback> control_signal_callbacks_
      ABSL_GUARDED_BY(control_signal_callbacks_mutex_);

  int64_t first_empty_keypoints_timestamp_us_ = 0;
};

}  // namespace guideline::environment

#endif  // PROJECT_GUIDELINE_VISION_GUIDANCE_SYSTEM_H_
