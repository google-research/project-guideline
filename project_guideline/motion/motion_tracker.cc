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

#include "project_guideline/motion/motion_tracker.h"

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "project_guideline/camera/camera_model.h"

namespace guideline::motion {

std::shared_ptr<camera::CameraModel> MotionTracker::camera_model() {
  absl::MutexLock lock(&mutex_);
  return camera_model_;
}

void MotionTracker::AddCameraMotionCallback(
    const CameraMotionCallback& callback) {
  absl::MutexLock lock(&mutex_);
  camera_motion_callbacks_.push_back(callback);
}

void MotionTracker::AddTrackingStateCallback(
    const TrackingStateCallback& callback) {
  absl::MutexLock lock(&mutex_);
  tracking_state_callbacks_.push_back(callback);
}

void MotionTracker::AddDepthMapCallback(const DepthMapCallback& callback) {
  absl::MutexLock lock(&mutex_);
  depth_map_callbacks_.push_back(callback);
}

void MotionTracker::AddTrackingFeaturesCallback(
    const TrackingFeaturesCallback& callback) {
  absl::MutexLock lock(&mutex_);
  tracking_features_callbacks_.push_back(callback);
}

bool MotionTracker::IsTracking() {
  absl::MutexLock lock(&mutex_);
  return is_tracking_;
}

std::optional<std::pair<int64_t, util::Transformation>>
MotionTracker::LastCameraPose() {
  absl::MutexLock lock(&mutex_);
  return last_camera_pose_;
}

void MotionTracker::UpdateCameraModel(
    std::shared_ptr<camera::CameraModel> camera_model) {
  absl::MutexLock lock(&mutex_);
  camera_model_ = camera_model;
}

void MotionTracker::NotifyTracking(bool is_tracking) {
  std::vector<TrackingStateCallback> callbacks;
  {
    absl::MutexLock lock(&mutex_);
    is_tracking_ = is_tracking;
    last_camera_pose_ = std::nullopt;
    callbacks = tracking_state_callbacks_;
  }

  for (const auto& callback : callbacks) {
    callback(is_tracking);
  }
}

void MotionTracker::NotifyCameraMotion(
    const int64_t timestamp_us, const util::Transformation& world_t_camera) {
  std::vector<CameraMotionCallback> callbacks;
  std::shared_ptr<camera::CameraModel> camera_model;
  {
    absl::MutexLock lock(&mutex_);
    last_camera_pose_ = std::pair(timestamp_us, world_t_camera);
    callbacks = camera_motion_callbacks_;
    camera_model = camera_model_;
  }

  CHECK(camera_model != nullptr) << "Camera model must be set first";

  for (const auto& callback : callbacks) {
    callback(timestamp_us, world_t_camera, camera_model);
  }
}

void MotionTracker::NotifyDepth(int64_t timestamp_us,
                                const util::DepthImageU16& depth_image,
                                const util::ConfidenceImageU8& confidence) {
  std::vector<DepthMapCallback> callbacks;
  {
    absl::MutexLock lock(&mutex_);
    callbacks = depth_map_callbacks_;
  }

  for (const DepthMapCallback& callback : callbacks) {
    callback(timestamp_us, depth_image, confidence);
  }
}

void MotionTracker::NotifyTrackingFeatures(
    const int64_t timestamp_us, const std::vector<TrackingFeature>& features) {
  std::vector<TrackingFeaturesCallback> callbacks;
  {
    absl::MutexLock lock(&mutex_);
    callbacks = tracking_features_callbacks_;
  }

  for (const TrackingFeaturesCallback& callback : callbacks) {
    callback(timestamp_us, features);
  }
}

}  //  namespace guideline::motion
