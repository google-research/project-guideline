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

#include "project_guideline/motion/passthrough_motion_tracker.h"

#include <optional>
#include <utility>

#include "absl/functional/bind_front.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"

namespace guideline::motion {

void PassthroughMotionEmitter::OnTracking(const bool is_tracking) {
  absl::MutexLock lock(&mutex_);
  if (tracking_callback_.has_value()) {
    (*tracking_callback_)(is_tracking);
  }
}

void PassthroughMotionEmitter::OnCameraPose(
    const int64_t timestamp_us, const util::Transformation& world_t_camera) {
  absl::MutexLock lock(&mutex_);
  if (motion_callback_.has_value()) {
    (*motion_callback_)(timestamp_us, world_t_camera);
  }
}

void PassthroughMotionEmitter::SetCallbacks(
    const PassthroughMotionCallback& motion_callback,
    const motion::TrackingStateCallback& tracking_callback) {
  absl::MutexLock lock(&mutex_);
  motion_callback_ = motion_callback;
  tracking_callback_ = tracking_callback;
}

void PassthroughMotionEmitter::ClearCallbacks() {
  absl::MutexLock lock(&mutex_);
  motion_callback_ = std::nullopt;
  tracking_callback_ = std::nullopt;
}

absl::Status PassthroughMotionTracker::Start() {
  emitter_->SetCallbacks(
      absl::bind_front(&PassthroughMotionTracker::OnCameraPose, this),
      absl::bind_front(&PassthroughMotionTracker::OnTracking, this));
  return absl::OkStatus();
}

absl::Status PassthroughMotionTracker::Stop() {
  emitter_->ClearCallbacks();
  return absl::OkStatus();
}

void PassthroughMotionTracker::OnTracking(const bool is_tracking) {
  {
    absl::MutexLock lock(&mutex_);
    if (tracking_ != is_tracking) {
      tracking_ = is_tracking;
    }
  }

  GuidelineLogEvent event;
  event.mutable_tracking_state_changed()->set_tracking(is_tracking);
  logger_->LogEvent(std::move(event));

  NotifyTracking(is_tracking);
}

void PassthroughMotionTracker::OnCameraPose(
    const int64_t timestamp_us, const util::Transformation& world_t_camera) {
  GuidelineLogEvent event;
  event.set_frame_timestamp_us(timestamp_us);
  auto pose_update = event.mutable_camera_pose_updated();
  auto transformation = pose_update->mutable_transformation();
  transformation->mutable_translation()->set_x(world_t_camera.p().x());
  transformation->mutable_translation()->set_y(world_t_camera.p().y());
  transformation->mutable_translation()->set_z(world_t_camera.p().z());
  transformation->mutable_rotation()->set_w(world_t_camera.q().w());
  transformation->mutable_rotation()->set_x(world_t_camera.q().x());
  transformation->mutable_rotation()->set_y(world_t_camera.q().y());
  transformation->mutable_rotation()->set_z(world_t_camera.q().z());
  logger_->LogEvent(std::move(event));

  NotifyCameraMotion(timestamp_us, world_t_camera);
}

}  // namespace guideline::motion
