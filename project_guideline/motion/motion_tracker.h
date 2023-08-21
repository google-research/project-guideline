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

#ifndef PROJECT_GUIDELINE_MOTION_MOTION_TRACKER_H_
#define PROJECT_GUIDELINE_MOTION_MOTION_TRACKER_H_

#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "project_guideline/camera/camera_model.h"
#include "project_guideline/motion/tracking_feature.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/transformation.h"

namespace guideline::motion {

using CameraMotionCallback = std::function<void(
    const int64_t timestamp_us, const util::Transformation& world_t_camera,
    std::shared_ptr<camera::CameraModel> camera_model)>;

using TrackingStateCallback = std::function<void(const bool is_tracking)>;

using DepthMapCallback = std::function<void(
    const int64_t timestamp_us, const util::DepthImageU16& depth_image,
    const util::ConfidenceImageU8& confidence)>;

using TrackingFeaturesCallback = std::function<void(
    int64_t timestamp_us, const std::vector<TrackingFeature>& features)>;

class MotionTracker {
 public:
  MotionTracker() = default;
  explicit MotionTracker(std::shared_ptr<camera::CameraModel> camera_model)
      : camera_model_(camera_model) {}
  virtual ~MotionTracker() = default;

  virtual absl::Status Start() = 0;
  virtual absl::Status Stop() = 0;

  // Gets the latest CameraModel. May be nullptr until the first
  // CameraMotionCallback is invoked.
  std::shared_ptr<camera::CameraModel> camera_model();

  void AddCameraMotionCallback(const CameraMotionCallback& callback);
  void AddTrackingStateCallback(const TrackingStateCallback& callback);
  void AddDepthMapCallback(const DepthMapCallback& callback);
  void AddTrackingFeaturesCallback(const TrackingFeaturesCallback& callback);

  bool IsTracking();
  std::optional<std::pair<int64_t, util::Transformation>> LastCameraPose();

 protected:
  // Updates the camera model. Must be called by subclasses before
  // NotifyCameraMotion() if a valid CameraModel was not passed in the
  // constructor.
  void UpdateCameraModel(std::shared_ptr<camera::CameraModel> camera_model);
  void NotifyTracking(bool is_tracking);
  void NotifyCameraMotion(int64_t timestamp_us,
                          const util::Transformation& world_t_camera);
  void NotifyDepth(int64_t timestamp_us, const util::DepthImageU16& depth_image,
                   const util::ConfidenceImageU8& confidence);
  void NotifyTrackingFeatures(int64_t timestamp_us,
                              const std::vector<TrackingFeature>& features);

 private:
  absl::Mutex mutex_;
  std::shared_ptr<camera::CameraModel> camera_model_ ABSL_GUARDED_BY(mutex_) =
      nullptr;
  std::vector<CameraMotionCallback> camera_motion_callbacks_
      ABSL_GUARDED_BY(mutex_);
  std::vector<TrackingStateCallback> tracking_state_callbacks_
      ABSL_GUARDED_BY(mutex_);
  std::vector<DepthMapCallback> depth_map_callbacks_ ABSL_GUARDED_BY(mutex_);
  std::vector<TrackingFeaturesCallback> tracking_features_callbacks_
      ABSL_GUARDED_BY(mutex_);
  bool is_tracking_ ABSL_GUARDED_BY(mutex_) = false;
  std::optional<std::pair<int64_t, util::Transformation>> ABSL_GUARDED_BY(
      mutex_) last_camera_pose_ = std::nullopt;
};

}  // namespace guideline::motion

#endif  // PROJECT_GUIDELINE_MOTION_MOTION_TRACKER_H_
