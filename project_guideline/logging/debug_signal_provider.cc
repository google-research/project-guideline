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

#include "project_guideline/logging/debug_signal_provider.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "Eigen/Core"
#include "project_guideline/camera/camera_model.h"
#include "project_guideline/environment/control_signal.h"
#include "project_guideline/logging/debug_signal.h"
#include "project_guideline/motion/tracking_feature.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/transformation.h"

namespace guideline::logging {

absl::StatusOr<std::unique_ptr<DebugSignalProvider>>
DebugSignalProvider::Create() {
  return absl::WrapUnique(new DebugSignalProvider());
}

DebugSignalProvider::DebugSignalProvider() {}

DebugSignalProvider::~DebugSignalProvider() {}

absl::Status DebugSignalProvider::Start() { return absl::OkStatus(); }

absl::Status DebugSignalProvider::Stop() { return absl::OkStatus(); }

void DebugSignalProvider::RunDebugSignalProvider() {
  while (running_) {
    auto timestamp = absl::Now();
    if ((timestamp - last_debug_timestamp) >=
        absl::Milliseconds(DEBUG_PERIOD_MS)) {
      NotifyDebugSignalCallbacks();
      last_debug_timestamp = timestamp;
    }
  }
}

void DebugSignalProvider::OnCameraPose(
    int64_t timestamp_us, const util::Transformation& world_t_camera,
    std::shared_ptr<camera::CameraModel> camera_model) {
  {
    absl::MutexLock lock(&mutex_);
    if (timestamp_us - last_camera_pose_timestamp_us_ < pow(10, 6)) {
      camera_pose_update_count_++;
    } else {
      current_camera_pose_fps_ = camera_pose_update_count_;
      last_camera_pose_timestamp_us_ = timestamp_us;
      camera_pose_update_count_ = 0;
    }
    camera_pose_lag_count_ = 0;
  }
}

void DebugSignalProvider::OnDetection(
    int64_t timestamp_us, const std::vector<Eigen::Vector3f>& keypoints,
    std::shared_ptr<const util::ConfidenceMask> guideline_mask,
    std::shared_ptr<const util::DepthImage> depth_map) {
  {
    absl::MutexLock lock(&mutex_);
    if (timestamp_us - last_detection_timestamp_us_ < pow(10, 6)) {
      detection_update_count_++;
    } else {
      current_detection_fps_ = detection_update_count_;
      last_detection_timestamp_us_ = timestamp_us;
      detection_update_count_ = 0;
    }
    detection_lag_count_ = 0;
  }
  NotifyDebugSignalCallbacks();
}

void DebugSignalProvider::OnControlSignal(
    const environment::ControlSignal& control_signal) {}

void DebugSignalProvider::OnGuideline(
    const std::vector<Eigen::Vector3d>& guideline) {
  {
    absl::MutexLock lock(&mutex_);
    guideline_count_ = guideline.size();
  }
}

void DebugSignalProvider::OnTrackingStateChanged(bool is_tracking) {
  {
    absl::MutexLock lock(&mutex_);
    tracking_ = is_tracking;
  }
}

void DebugSignalProvider::OnTrackingFeatures(
    int64_t timestamp_us,
    const std::vector<motion::TrackingFeature>& features) {
  {
    absl::MutexLock lock(&mutex_);
    if (timestamp_us - last_tracking_features_timestamp_us_ < 1000000) {
      tracking_features_update_count_++;
    } else {
      current_tracking_features_fps_ = tracking_features_update_count_;
      last_tracking_features_timestamp_us_ = timestamp_us;
      tracking_features_update_count_ = 0;
    }
    tracking_features_lag_count_ = 0;
  }
}

void DebugSignalProvider::AddDebugSignalCallback(
    const DebugSignalCallback& debug_signal_callback) {
  {
    absl::MutexLock lock(&mutex_);
    debug_signal_callbacks_.push_back(debug_signal_callback);
  }
}

void DebugSignalProvider::NotifyDebugSignalCallbacks() {
  DebugSignal debug_signal;
  {
    absl::MutexLock lock(&mutex_);
    debug_signal.camera_pose_fps = current_camera_pose_fps_;
    debug_signal.camera_pose_lagging_frame = camera_pose_lag_count_;
    camera_pose_lag_count_++;
    debug_signal.detection_fps = current_detection_fps_;
    debug_signal.detection_lagging_frame = detection_lag_count_;
    detection_lag_count_++;
    debug_signal.tracking_features_fps = current_tracking_features_fps_;
    debug_signal.tracking_features_lagging_frame = tracking_features_lag_count_;
    tracking_features_lag_count_++;
    debug_signal.tracking = tracking_;
    for (const auto& callback : debug_signal_callbacks_) {
      callback(debug_signal);
    }
  }
}
}  // namespace guideline::logging
