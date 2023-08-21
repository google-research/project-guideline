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

#ifndef PROJECT_GUIDELINE_UNREAL_UNREAL_PLUGIN_IMPL_H_
#define PROJECT_GUIDELINE_UNREAL_UNREAL_PLUGIN_IMPL_H_

#include <atomic>
#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "project_guideline/data/passthrough_data_source.h"
#include "project_guideline/engine/guideline_engine.h"
#include "project_guideline/environment/control_signal.h"
#include "project_guideline/logging/guideline_logger.h"
#include "project_guideline/motion/passthrough_motion_tracker.h"
#include "project_guideline/unreal/unreal_audio_output_stream.h"
#include "project_guideline/unreal/unreal_plugin.h"

namespace guideline::unreal {

class UnrealPluginImpl : public UnrealPlugin {
 public:
  static absl::StatusOr<std::unique_ptr<UnrealPluginImpl>> Create(
      const UnrealGuidelineOptions& options);

  ~UnrealPluginImpl() override;

  void Start() override;
  void Stop() override;

  void SetControlSignalListener(UnrealControlSignalListener* listener) override;

  void SetDebugLogListener(UnrealDebugLogListener* listener) override;

  void SetLogEventCallback(const logging::LogEventCallback& callback);

  void OnTracking(bool tracking) override;

  void OnCameraPose(uint64_t timestamp_us, float tx, float ty, float tz,
                    float qx, float qy, float qz, float qw) override;

  void OnImage(uint64_t timestamp_us, uint16_t width, uint16_t height,
               void* data) override;

  int32_t OnGenerateAudio(int16_t* out_buffer, int32_t num_samples) override;

  GuidelineRunnerPose GetRunnerPose() override;

  void GetGuidelinePoints(GuidelinePoints* out) override;
  void ReleaseGuidelinePoints(GuidelinePoints* points) override;

  UnrealControlSignal GetLastControlSignal() override;

  void GetCameraImageDimensions(float& width, float& height) override;

  void GetProjectionMatrix(float near, float far,
                           float* out_col_major_4x4) override;

  engine::GuidelineEngine& guideline_engine();

 private:
  explicit UnrealPluginImpl(
      std::unique_ptr<engine::GuidelineEngine> engine,
      std::shared_ptr<data::PassthroughDataEmitter> data_emitter,
      std::shared_ptr<motion::PassthroughMotionEmitter> motion_emitter,
      std::shared_ptr<UnrealAudioOutputStream> audio_output_stream,
      std::shared_ptr<logging::GuidelineLogger> logger,
      std::shared_ptr<camera::CameraModel> camera_model);

  absl::Status Initialize();

  void OnControlSignal(const environment::ControlSignal& control_signal);

  void OnDebugMessage(logging::DebugLogLevel level, const std::string& message);

  std::atomic<bool> started_;
  std::unique_ptr<engine::GuidelineEngine> engine_;
  std::shared_ptr<data::PassthroughDataEmitter> data_emitter_;
  std::shared_ptr<motion::PassthroughMotionEmitter> motion_emitter_;
  std::shared_ptr<UnrealAudioOutputStream> audio_output_stream_;
  std::shared_ptr<logging::GuidelineLogger> logger_;
  std::shared_ptr<camera::CameraModel> camera_model_;

  absl::Mutex mutex_;
  UnrealControlSignalListener* control_signal_listener_
      ABSL_GUARDED_BY(mutex_) = nullptr;
  UnrealDebugLogListener* debug_log_listener_ ABSL_GUARDED_BY(mutex_) = nullptr;

  UnrealControlSignal last_control_signal_ ABSL_GUARDED_BY(mutex_);
};

}  // namespace guideline::unreal

#endif  // PROJECT_GUIDELINE_UNREAL_UNREAL_PLUGIN_IMPL_H_
