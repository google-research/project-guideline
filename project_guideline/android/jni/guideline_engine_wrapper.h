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

#ifndef PROJECT_GUIDELINE_ANDROID_JNI_GUIDELINE_ENGINE_WRAPPER_H_
#define PROJECT_GUIDELINE_ANDROID_JNI_GUIDELINE_ENGINE_WRAPPER_H_

#include <jni.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "project_guideline/android/arcore/arcore_data_source.h"
#include "project_guideline/android/arcore/arcore_motion_tracker.h"
#include "project_guideline/android/arcore/arcore_session.h"
#include "project_guideline/android/jni/jni_helpers.h"
#include "project_guideline/engine/guideline_engine.h"
#include "project_guideline/environment/control_signal.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/visualization/camera_feed_renderer.h"
#include "project_guideline/visualization/environment_map_renderer.h"

namespace guideline {

class ControlSignalCallbackWrapper {
 public:
  virtual ~ControlSignalCallbackWrapper() = default;
  virtual void OnControlSignal(float rotation_degrees,
                               float lateral_distance_meters, bool stop) = 0;
};

class GuidelineEngineWrapper {
 public:
  static absl::StatusOr<std::unique_ptr<GuidelineEngineWrapper>> Create(
      jobject app_context, const std::string& build_fingerprint,
      const GuidelineEngineConfig& config,
      size_t preferred_audio_frames_per_buffer,
      size_t preferred_audio_sample_rate_hz);

  ~GuidelineEngineWrapper();

  absl::Status Start();
  absl::Status Stop();
  absl::Status SetPreviewTexture(int texture_id);

  absl::Status StartRecording(std::string file_path);
  absl::Status StopRecording();

  void OnGlSurfaceCreated();
  void OnGlViewportChanged(int width, int height);
  void OnGlDrawFrame();

  void SetControlSignalCallback(
      std::unique_ptr<ControlSignalCallbackWrapper> callback);

  void OnBatteryLevel(int level);

 private:
  GuidelineEngineWrapper(
      jobject app_context,
      std::shared_ptr<arcore::ArcoreSession> arcore_session,
      std::unique_ptr<engine::GuidelineEngine> guideline_engine,
      const GuidelineEngineConfig& config);

  absl::Status Initialize();

  void OnControlSignal(const environment::ControlSignal& control_signal);

  jni::WeakGlobalRef app_context_;
  std::shared_ptr<arcore::ArcoreSession> arcore_session_;
  std::unique_ptr<engine::GuidelineEngine> guideline_engine_;

  visualization::CameraFeedRenderer camera_feed_renderer_;
  visualization::EnvironmentMapRenderer environment_map_renderer_;

  absl::Mutex control_signal_callback_mutex_;
  std::unique_ptr<ControlSignalCallbackWrapper> control_signal_calback_
      ABSL_GUARDED_BY(control_signal_callback_mutex_) = nullptr;

  absl::Mutex texture_id_mutex_;
  std::optional<int> texture_id_ ABSL_GUARDED_BY(texture_id_mutex_);
};

}  // namespace guideline

#endif  // PROJECT_GUIDELINE_ANDROID_JNI_GUIDELINE_ENGINE_WRAPPER_H_
