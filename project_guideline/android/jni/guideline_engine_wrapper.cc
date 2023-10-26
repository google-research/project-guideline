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

#include "project_guideline/android/jni/guideline_engine_wrapper.h"

#include <jni.h>

#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/functional/bind_front.h"
#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "Eigen/Core"
#include "project_guideline/android/arcore/arcore_data_source.h"
#include "project_guideline/android/arcore/arcore_motion_tracker.h"
#include "project_guideline/android/arcore/arcore_session.h"
#include "project_guideline/android/audio/android_audio_output_stream.h"
#include "project_guideline/android/jni/jni_helpers.h"
#include "project_guideline/audio/audio_output_stream.h"
#include "project_guideline/camera/camera_model.h"
#include "project_guideline/environment/control_signal.h"
#include "project_guideline/logging/guideline_logger.h"
#include "project_guideline/logging/noop_guideline_logger.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/status.h"

namespace guideline {

namespace {
using arcore::ArcoreDataSource;
using arcore::ArcoreMotionTracker;
using arcore::ArcoreSession;
using audio::AndroidAudioOutputStream;
using audio::AudioOutputStream;
using engine::GuidelineEngine;
using jni::EnvRef;
using logging::GuidelineLogger;
using logging::NoopGuidelineLogger;
using visualization::EnvironmentMapRenderer;

// Gets approximate rear camera distortion coefficients for the device. The
// distortion coefficients help improve accuracy but are not available through
// ARCore, so this returns hardcoded average values for supported devices
// (Pixel devices with Tensor chip).
// These values can be determined by performing camera calibration, see
// https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html.
Eigen::Vector3f GetDeviceDistortionCoefficients(std::string build_fingerprint) {
  std::string::size_type pos = build_fingerprint.find(':');
  if (pos == std::string::npos) {
    LOG(WARNING) << "Could not determine device type: " << build_fingerprint;
    return Eigen::Vector3f::Zero();
  }
  std::string device_id = build_fingerprint.substr(0, pos);
  if (device_id == "google/panther/panther") {
    // Pixel 7
    return {0.08778850, -0.12531260, 0.06413092};
  } else if (device_id == "google/cheetah/cheetah") {
    // Pixel 7 Pro
    return {0.08742616, -0.12614126, 0.06744798};
  } else if (device_id == "google/lynx/lynx") {
    // Pixel 7a
    return {0.09055725, -0.18180295, 0.15791714};
  } else if (device_id == "google/oriole/oriole") {
    //  Pixel 6
    return {0.09710243, -0.14840230, 0.08555396};
  } else if (device_id == "google/raven/raven") {
    //  Pixel 6 XL
    return {0.08152215, -0.11745508, 0.06526846};
  } else if (device_id == "google/bluejay/bluejay") {
    // Pixel 6a
    return {0.08779683, -0.20808211, 0.14967772};
  }

  LOG(WARNING) << "Distortion coefficients unavailable for device: "
               << device_id;
  return Eigen::Vector3f::Zero();
}

}  // namespace

absl::StatusOr<std::unique_ptr<GuidelineEngineWrapper>>
GuidelineEngineWrapper::Create(jobject app_context,
                               const std::string& build_fingerprint,
                               const GuidelineEngineConfig& config,
                               size_t preferred_audio_frames_per_buffer,
                               size_t preferred_audio_sample_rate_hz) {
  std::shared_ptr<GuidelineLogger> logger =
      std::make_unique<NoopGuidelineLogger>();

  constexpr size_t kMinFramesPerBufferSize = 256;
  const size_t frames_per_buffer =
      std::max(kMinFramesPerBufferSize, preferred_audio_frames_per_buffer);

  GL_ASSIGN_OR_RETURN(std::shared_ptr<AudioOutputStream> audio_output_stream,
                      AndroidAudioOutputStream::Create(
                          frames_per_buffer, preferred_audio_sample_rate_hz));

  auto arcore_session = std::make_shared<ArcoreSession>(
      GetDeviceDistortionCoefficients(build_fingerprint));
  auto data_source = std::make_shared<ArcoreDataSource>(arcore_session);
  auto motion_tracker = std::make_shared<ArcoreMotionTracker>(arcore_session);

  GL_ASSIGN_OR_RETURN(
      auto guideline_engine,
      GuidelineEngine::Create(config, data_source, motion_tracker,
                              audio_output_stream, logger));

  auto engine_wrapper = absl::WrapUnique(new GuidelineEngineWrapper(
      app_context, arcore_session, std::move(guideline_engine), config));
  GL_RETURN_IF_ERROR(engine_wrapper->Initialize());
  return engine_wrapper;
}

GuidelineEngineWrapper::GuidelineEngineWrapper(
    jobject app_context, std::shared_ptr<ArcoreSession> arcore_session,
    std::unique_ptr<engine::GuidelineEngine> guideline_engine,
    const GuidelineEngineConfig& config)
    : app_context_(app_context),
      arcore_session_(arcore_session),
      guideline_engine_(std::move(guideline_engine)),
      camera_feed_renderer_(/*use_gl_texture=*/true,
                            /*texture_rotation_degrees=*/180,
                            /*texture_aspect_ratio=*/16. / 9.) {}

GuidelineEngineWrapper::~GuidelineEngineWrapper() {}

absl::Status GuidelineEngineWrapper::Initialize() {
  CHECK_OK(guideline_engine_->data_source().SetGlTextureOutput(
      [this]() { camera_feed_renderer_.OnGlTextureUpdated(); }));

  // Set up callbacks for the EnvironmentMapRenderer.
  guideline_engine_->motion_tracker().AddCameraMotionCallback(
      [this](const int64_t timestamp_us,
             const util::Transformation& world_t_camera,
             std::shared_ptr<camera::CameraModel> camera_model) {
        auto position_direction =
            guideline_engine_->guidance_system().CurrentPositionAndDirection();
        if (position_direction.ok()) {
          environment_map_renderer_.OnPose(*position_direction);
        }
      });

  guideline_engine_->guideline_detector().AddCallback(
      [this](const int64_t timestamp_ns,
             const std::vector<Eigen::Vector3f>& keypoints,
             std::shared_ptr<const util::ConfidenceMask> guideline_mask,
             std::shared_ptr<const util::DepthImage> depth_map) {
        auto guideline = guideline_engine_->guidance_system().GetGuideline();
        if (guideline.ok()) {
          environment_map_renderer_.OnGuideline(*guideline);
        } else {
          environment_map_renderer_.OnGuideline({});
        }
        auto obstacles = guideline_engine_->guidance_system().GetObstacles();
        if (obstacles.ok()) {
          environment_map_renderer_.OnObstacles(*obstacles);
        } else {
          environment_map_renderer_.OnObstacles({});
        }
      });

  guideline_engine_->guidance_system().AddControlSignalCallback(
      absl::bind_front(&EnvironmentMapRenderer::OnControlSignal,
                       &environment_map_renderer_));

  return absl::OkStatus();
}

absl::Status GuidelineEngineWrapper::Start() {
  absl::MutexLock lock(&texture_id_mutex_);
  if (texture_id_) {
    arcore_session_->SetGlCameraTexture(*texture_id_);
  }
  GL_RETURN_IF_ERROR(guideline_engine_->Start());
  GL_RETURN_IF_ERROR(
      arcore_session_->OnResume(EnvRef::GetEnv(), *app_context_));

  return absl::OkStatus();
}

absl::Status GuidelineEngineWrapper::Stop() {
  GL_RETURN_IF_ERROR(guideline_engine_->Stop());
  GL_RETURN_IF_ERROR(arcore_session_->OnPause());

  camera_feed_renderer_.OnGlTextureReset();
  environment_map_renderer_.Reset();

  return absl::OkStatus();
}

absl::Status GuidelineEngineWrapper::SetPreviewTexture(int texture_id) {
  absl::MutexLock lock(&texture_id_mutex_);
  texture_id_ = texture_id;
  arcore_session_->SetGlCameraTexture(texture_id);
  return absl::OkStatus();
}

void GuidelineEngineWrapper::OnGlSurfaceCreated() {
  CHECK_OK(camera_feed_renderer_.OnGlInit())
      << "Failed to initialize CameraFeedRenderer";
  CHECK_OK(guideline_engine_->data_source().GlAttachTexture(
      camera_feed_renderer_.gl_texture_id()))
      << "Failed to attach texture";
  environment_map_renderer_.InitGL();
}

void GuidelineEngineWrapper::OnGlViewportChanged(int width, int height) {
  arcore_session_->SetViewport(width, height);
  camera_feed_renderer_.SetViewport(width, height);
  environment_map_renderer_.SetViewport(width, height);
}

void GuidelineEngineWrapper::OnGlDrawFrame() {
  arcore_session_->OnDrawFrame();
  camera_feed_renderer_.Render();
  environment_map_renderer_.Render();
}

void GuidelineEngineWrapper::OnBatteryLevel(int level) {
  guideline_engine_->OnBatteryLevel(level);
}

void GuidelineEngineWrapper::SetControlSignalCallback(
    std::unique_ptr<ControlSignalCallbackWrapper> callback) {
  absl::MutexLock lock(&control_signal_callback_mutex_);
  control_signal_calback_ = std::move(callback);
}

void GuidelineEngineWrapper::OnControlSignal(
    const environment::ControlSignal& control_signal) {
  absl::MutexLock lock(&control_signal_callback_mutex_);
  if (control_signal_calback_) {
    control_signal_calback_->OnControlSignal(
        control_signal.rotation_movement_degrees,
        control_signal.lateral_movement_meters, control_signal.stop);
  }
}

absl::Status GuidelineEngineWrapper::StartRecording(std::string file_path) {
  // TODO(b/276738891): Implement recording
  return absl::UnimplementedError("Recording not implemented yet");
}

absl::Status GuidelineEngineWrapper::StopRecording() {
  // TODO(b/276738891): Implement recording
  return absl::UnimplementedError("Recording not implemented yet");
}

}  // namespace guideline
