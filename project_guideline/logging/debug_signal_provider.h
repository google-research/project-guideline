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

#ifndef PROJECT_GUIDELINE_LOGGING_DEBUG_SIGNAL_PROVIDER_H_
#define PROJECT_GUIDELINE_LOGGING_DEBUG_SIGNAL_PROVIDER_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
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

class DebugSignalProvider {
 public:
  using DebugSignalCallback =
      std::function<void(const DebugSignal& debug_signal)>;
  static absl::StatusOr<std::unique_ptr<DebugSignalProvider>> Create();

  DebugSignalProvider& operator=(const DebugSignalProvider&) = delete;
  ~DebugSignalProvider();

  void OnCameraPose(int64_t timestamp_us,
                    const util::Transformation& world_t_camera,
                    std::shared_ptr<camera::CameraModel> camera_model);
  void OnDetection(int64_t timestamp_us,
                   const std::vector<Eigen::Vector3f>& keypoints,
                   std::shared_ptr<const util::ConfidenceMask> guideline_mask,
                   std::shared_ptr<const util::DepthImage> depth_map);

  void OnControlSignal(const environment::ControlSignal& control_signal);
  void OnGuideline(const std::vector<Eigen::Vector3d>& guideline);
  void OnTrackingStateChanged(bool is_tracking);
  void OnTrackingFeatures(int64_t timestamp_us,
                          const std::vector<motion::TrackingFeature>& features);

  void AddDebugSignalCallback(const DebugSignalCallback& debug_signal_callback)
      ABSL_LOCKS_EXCLUDED(mutex_);

  absl::Status Start();
  absl::Status Stop();

 private:
  explicit DebugSignalProvider();
  void NotifyDebugSignalCallbacks() ABSL_LOCKS_EXCLUDED(mutex_);
  void RunDebugSignalProvider();

 private:
  absl::Mutex mutex_;
  int64_t last_camera_pose_timestamp_us_ ABSL_GUARDED_BY(mutex_) = 0;
  int64_t camera_pose_update_count_ ABSL_GUARDED_BY(mutex_) = 0;
  int64_t camera_pose_lag_count_ ABSL_GUARDED_BY(mutex_) = 0;
  int64_t current_camera_pose_fps_ ABSL_GUARDED_BY(mutex_) = 0;

  int64_t last_detection_timestamp_us_ ABSL_GUARDED_BY(mutex_) = 0;
  int64_t detection_update_count_ ABSL_GUARDED_BY(mutex_) = 0;
  int64_t detection_lag_count_ ABSL_GUARDED_BY(mutex_) = 0;
  int64_t current_detection_fps_ ABSL_GUARDED_BY(mutex_) = 0;

  int64_t last_tracking_features_timestamp_us_ ABSL_GUARDED_BY(mutex_) = 0;
  int64_t tracking_features_update_count_ ABSL_GUARDED_BY(mutex_) = 0;
  int64_t tracking_features_lag_count_ ABSL_GUARDED_BY(mutex_) = 0;
  int64_t current_tracking_features_fps_ ABSL_GUARDED_BY(mutex_) = 0;

  bool tracking_ ABSL_GUARDED_BY(mutex_) = false;

  int64_t guideline_count_ ABSL_GUARDED_BY(mutex_) = 0;

  std::vector<DebugSignalCallback> debug_signal_callbacks_
      ABSL_GUARDED_BY(mutex_);

  std::atomic<bool> running_ = false;
  std::unique_ptr<std::thread> debug_thread_;

  absl::Time last_debug_timestamp = absl::Now();

  const int64_t DEBUG_PERIOD_MS = 500;
};
}  // namespace guideline::logging

#endif  // PROJECT_GUIDELINE_LOGGING_DEBUG_SIGNAL_PROVIDER_H_
