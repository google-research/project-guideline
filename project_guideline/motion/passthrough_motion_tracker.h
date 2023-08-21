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

#ifndef PROJECT_GUIDELINE_MOTION_PASSTHROUGH_MOTION_TRACKER_H_
#define PROJECT_GUIDELINE_MOTION_PASSTHROUGH_MOTION_TRACKER_H_

#include <memory>
#include <optional>

#include "project_guideline/camera/camera_model.h"
#include "project_guideline/logging/guideline_logger.h"
#include "project_guideline/motion/motion_tracker.h"
#include "project_guideline/util/transformation.h"

namespace guideline::motion {

using PassthroughMotionCallback = std::function<void(
    const int64_t timestamp_us, const util::Transformation& world_t_camera)>;

class PassthroughMotionEmitter {
 public:
  PassthroughMotionEmitter() = default;

  void OnTracking(bool is_tracking);

  void OnCameraPose(int64_t timestamp_us,
                    const util::Transformation& world_t_camera);

 private:
  friend class PassthroughMotionTracker;
  void SetCallbacks(const PassthroughMotionCallback& motion_callback,
                    const motion::TrackingStateCallback& tracking_callback);
  void ClearCallbacks();

  absl::Mutex mutex_;
  std::optional<PassthroughMotionCallback> motion_callback_
      ABSL_GUARDED_BY(mutex_) = std::nullopt;
  std::optional<motion::TrackingStateCallback> tracking_callback_
      ABSL_GUARDED_BY(mutex_) = std::nullopt;
};

class PassthroughMotionTracker : public motion::MotionTracker {
 public:
  explicit PassthroughMotionTracker(
      std::shared_ptr<PassthroughMotionEmitter> emitter,
      std::shared_ptr<camera::CameraModel> camera_model,
      std::shared_ptr<logging::GuidelineLogger> logger)
      : MotionTracker(camera_model), emitter_(emitter), logger_(logger) {}

  ~PassthroughMotionTracker() override = default;

  absl::Status Start() override;
  absl::Status Stop() override;

 private:
  void OnTracking(bool is_tracking);

  void OnCameraPose(int64_t timestamp_us,
                    const util::Transformation& world_t_camera);

  std::shared_ptr<PassthroughMotionEmitter> emitter_;
  std::shared_ptr<logging::GuidelineLogger> logger_;

  absl::Mutex mutex_;
  bool tracking_ ABSL_GUARDED_BY(mutex_) = false;
};

}  // namespace guideline::motion

#endif  // PROJECT_GUIDELINE_MOTION_PASSTHROUGH_MOTION_TRACKER_H_
