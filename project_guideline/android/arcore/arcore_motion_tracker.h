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

#ifndef PROJECT_GUIDELINE_ANDROID_ARCORE_ARCORE_MOTION_TRACKER_H_
#define PROJECT_GUIDELINE_ANDROID_ARCORE_ARCORE_MOTION_TRACKER_H_

#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "Eigen/Core"
#include "project_guideline/android/arcore/arcore_session.h"
#include "project_guideline/motion/motion_tracker.h"

namespace guideline::arcore {

// MotionTracking implementation that uses ARCore NDK.
class ArcoreMotionTracker : public motion::MotionTracker {
 public:
  ArcoreMotionTracker(std::shared_ptr<ArcoreSession> arcore_session);

  absl::Status Start() override;
  absl::Status Stop() override;

 private:
  void OnFrameData(const ArcoreFrameData& frame_data);

  std::shared_ptr<ArcoreSession> arcore_session_;

  // The last camera pinhole parameters [fx, fy, cx, cy].
  Eigen::Vector4f last_camera_pinhole_params_;

  absl::Mutex mutex_;
  std::optional<int> frame_data_callback_key_ = std::nullopt;

  std::atomic<bool> tracking_ = false;
};

}  // namespace guideline::arcore

#endif  // PROJECT_GUIDELINE_ANDROID_ARCORE_ARCORE_MOTION_TRACKER_H_
