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

#include "project_guideline/unreal/unreal_plugin_impl.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "google/protobuf/duration.pb.h"
#include <opencv2/core.hpp>  // keep include
#include <opencv2/imgproc.hpp>
#include "absl/functional/bind_front.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "Eigen/Core"
#include "project_guideline/camera/camera_model.h"
#include "project_guideline/camera/camera_utils.h"
#include "project_guideline/camera/cv_camera_model.h"
#include "project_guideline/data/passthrough_data_source.h"
#include "project_guideline/logging/file_guideline_logger.h"
#include "project_guideline/logging/guideline_logger.h"
#include "project_guideline/logging/noop_guideline_logger.h"
#include "project_guideline/motion/passthrough_motion_tracker.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/unreal/unreal_plugin.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/lerp.h"
#include "project_guideline/util/status.h"
#include "project_guideline/util/transformation.h"

#define EXPORT __attribute__((visibility("default")))

namespace guideline::unreal {

namespace {
using data::PassthroughDataEmitter;
using data::PassthroughDataSource;
using engine::GuidelineEngine;
using logging::DebugLogLevel;
using logging::FileGuidelineLogger;
using logging::GuidelineLogger;
using logging::NoopGuidelineLogger;
using motion::PassthroughMotionEmitter;
using motion::PassthroughMotionTracker;
using util::Transformation;

// Image size for Pixel 4 rear camera params.
static const Eigen::Vector2i kPixel4ParamsImageSize(640, 480);

// Pinhole params for Pixel 4 rear camera.
static const Eigen::Vector4d kPixel4PinholeParams(506.44221032264841,   // fx
                                                  507.009274541992,     // fy
                                                  321.67192506889251,   // cx
                                                  240.15548821158652);  // cy

}  // namespace

EXPORT UnrealPlugin* CreateUnrealPlugin(const UnrealGuidelineOptions& options) {
  auto plugin = UnrealPluginImpl::Create(options);
  CHECK_OK(plugin.status());
  return plugin->release();
}

absl::StatusOr<std::unique_ptr<UnrealPluginImpl>> UnrealPluginImpl::Create(
    const UnrealGuidelineOptions& options) {
  std::shared_ptr<GuidelineLogger> logger;
  std::string file_logger_output_dir = options.file_logger_output_dir;
  if (file_logger_output_dir.empty()) {
    logger = std::make_unique<NoopGuidelineLogger>();
  } else {
    GL_ASSIGN_OR_RETURN(logger,
                        FileGuidelineLogger::Create(file_logger_output_dir,
                                                    options.log_intermediate,
                                                    DebugLogLevel::kVerbose));
  }

  // Create a CameraModel that matches a Pixel 4 rear camera. Note this is
  // using a pinhole model and no distortion params since the projection matrix
  // is a pinhole model so images from Unreal will not have any distortion.
  std::shared_ptr<camera::CameraModel> camera_model =
      std::make_shared<camera::CvCameraModel>(kPixel4ParamsImageSize.x(),
                                              kPixel4ParamsImageSize.y(),
                                              kPixel4PinholeParams);

  std::shared_ptr<PassthroughMotionEmitter> motion_emitter =
      std::make_unique<PassthroughMotionEmitter>();
  auto motion_tracker = std::make_unique<PassthroughMotionTracker>(
      motion_emitter, camera_model, logger);

  GuidelineEngineConfig config;
  auto keypoint_extractor_options =
      config.mutable_detector_options()->mutable_keypoint_extractor_options();
  (*keypoint_extractor_options
        ->mutable_horizontal_block_keypoint_extractor_options()) =
      HorizontalBlockKeypointExtractorOptions();

  if (options.enable_conservative_mode) {
    auto audio_system_options = config.mutable_audio_system_options();
    config.mutable_guidance_system_options()
        ->mutable_eager_stop_threshold()
        ->set_seconds(1);
    auto legacy_sound_pack_options =
        audio_system_options->mutable_legacy_sound_pack_options();
    legacy_sound_pack_options->set_type(LegacySoundPackOptions::V4_2);

    CHECK_GT(options.conservative_mode_lane_width_meters, 0)
        << "Invalid conservative_mode_lane_width_meters";

    // These values based on the legacy guidance app settings.
    const float kMinLaneWidthForRotation = 0.6;
    const float kMaxLaneWidthForRotation = 2.1;

    // These values computed by comparing rotation angle in legacy app with
    // various lane width settings.
    const float kMinLaneRotationDegrees = 20;
    const float kMaxLaneRotationDegrees = 45;
    legacy_sound_pack_options->set_warning_threshold_meters(
        options.conservative_mode_lane_width_meters);
    float max_rotation_degrees =
        util::ClampedLerp(options.conservative_mode_lane_width_meters,
                          kMinLaneWidthForRotation, kMaxLaneWidthForRotation,
                          kMinLaneRotationDegrees, kMaxLaneRotationDegrees);
    legacy_sound_pack_options->set_max_rotation_degrees(max_rotation_degrees);
  }

  CHECK_GT(options.audio_frames_per_buffer, 0)
      << "Invalid audio_frames_per_buffer";

  std::shared_ptr<UnrealAudioOutputStream> audio_output_stream =
      std::make_unique<UnrealAudioOutputStream>(
          options.audio_frames_per_buffer);

  // Using Guideline Aggregator V1.
  auto* guideline_aggregator_options =
      config.mutable_guideline_aggregator_options();
  (*guideline_aggregator_options
        ->mutable_local_temporal_regression_based_guideline_aggregator_options()) =  // NOLINT(whitespace/line_length)
      LocalTemporalRegressionBasedGuidelineAggregatorOptions();

  std::shared_ptr<PassthroughDataEmitter> data_emitter =
      std::make_unique<PassthroughDataEmitter>();
  auto data_source = std::make_unique<PassthroughDataSource>(data_emitter);

  GL_ASSIGN_OR_RETURN(std::unique_ptr<GuidelineEngine> engine,
                      GuidelineEngine::Create(config, std::move(data_source),
                                              std::move(motion_tracker),
                                              audio_output_stream, logger));

  auto plugin = absl::WrapUnique(
      new UnrealPluginImpl(std::move(engine), data_emitter, motion_emitter,
                           audio_output_stream, logger, camera_model));
  GL_RETURN_IF_ERROR(plugin->Initialize());

  return plugin;
}

UnrealPluginImpl::UnrealPluginImpl(
    std::unique_ptr<GuidelineEngine> engine,
    std::shared_ptr<PassthroughDataEmitter> data_emitter,
    std::shared_ptr<PassthroughMotionEmitter> motion_emitter,
    std::shared_ptr<UnrealAudioOutputStream> audio_output_stream,
    std::shared_ptr<GuidelineLogger> logger,
    std::shared_ptr<camera::CameraModel> camera_model)
    : engine_(std::move(engine)),
      data_emitter_(std::move(data_emitter)),
      motion_emitter_(std::move(motion_emitter)),
      audio_output_stream_(std::move(audio_output_stream)),
      logger_(std::move(logger)),
      camera_model_(std::move(camera_model)) {}

UnrealPluginImpl::~UnrealPluginImpl() {
  if (started_) {
    Stop();
  }
}

absl::Status UnrealPluginImpl::Initialize() {
  engine_->guidance_system().AddControlSignalCallback(
      absl::bind_front(&UnrealPluginImpl::OnControlSignal, this));
  logger_->SetDebugMessageCallback(
      absl::bind_front(&UnrealPluginImpl::OnDebugMessage, this));
  return absl::OkStatus();
}

void UnrealPluginImpl::Start() {
  CHECK_OK(engine_->Start());
  started_ = true;
}

void UnrealPluginImpl::Stop() {
  CHECK_OK(engine_->Stop());
  started_ = false;
}

void UnrealPluginImpl::OnTracking(bool tracking) {
  motion_emitter_->OnTracking(tracking);
}

void UnrealPluginImpl::OnCameraPose(uint64_t timestamp_us, float tx, float ty,
                                    float tz, float qx, float qy, float qz,
                                    float qw) {
  Transformation unrealWorld_T_unrealDisplayCamera({qw, qx, qy, qz},
                                                   {tx, ty, tz});

  // TODO: Precompute all these matrices so there's less work each time.

  // Unreal to GL is left-handed rotate 90 around Z, 90 around X, and flip Z.
  // The flip Z results in a right-handed coordinate system.
  Eigen::Affine3d a1 = Eigen::Affine3d::Identity();
  a1.prerotate(Eigen::AngleAxisd(90 * util::kDegreesToRadians,
                                 Eigen::Vector3d::UnitZ()));

  Eigen::Affine3d a2 = Eigen::Affine3d::Identity();
  a2.prerotate(Eigen::AngleAxisd(90 * util::kDegreesToRadians,
                                 Eigen::Vector3d::UnitX()));

  Eigen::Affine3d a3 = Eigen::Affine3d::Identity();
  a2.scale(Eigen::Vector3d(1., 1., -1.));

  Eigen::Matrix4d unrealWorld_T_glWorld =
      a1.matrix() * a2.matrix() * a3.matrix();
  Eigen::Matrix4d glWorld_T_unrealWorld = unrealWorld_T_glWorld.inverse();

  Transformation glWorld_T_glDisplayCamera = Transformation::FromMatrix(
      glWorld_T_unrealWorld * unrealWorld_T_unrealDisplayCamera.ToMatrix4x4() *
      unrealWorld_T_glWorld);

  // Converts from Display -> Device -> Physical for rear camera in
  // reverse landscape (Surface.ROTATION_270).
  Eigen::Affine3d affine_glDisplayCamera_Q_glPhysicalCamera =
      Eigen::Affine3d::Identity();
  affine_glDisplayCamera_Q_glPhysicalCamera.prerotate(Eigen::AngleAxisd(
      180 * util::kDegreesToRadians, Eigen::Vector3d::UnitZ()));
  Transformation glDisplayCamera_Q_glPhysicalCamera =
      Transformation::FromMatrix(
          affine_glDisplayCamera_Q_glPhysicalCamera.matrix());

  Transformation glWorld_T_glPhysicalCamera(glWorld_T_glDisplayCamera *
                                            glDisplayCamera_Q_glPhysicalCamera);

  // GL to MT is a right-handed -90 around around X
  Eigen::Affine3d affine_glWorld_Q_mtWorld = Eigen::Affine3d::Identity();
  affine_glWorld_Q_mtWorld.prerotate(Eigen::AngleAxisd(
      -90 * util::kDegreesToRadians, Eigen::Vector3d::UnitX()));
  Transformation glWorld_T_mtWorld =
      Transformation::FromMatrix(affine_glWorld_Q_mtWorld.matrix());

  // Handle difference between Open GL and internal Motion Tracking
  // camera conventions.
  Eigen::Affine3d affine_glCamera_Q_mtCamera = Eigen::Affine3d::Identity();
  affine_glCamera_Q_mtCamera.prerotate(Eigen::AngleAxisd(
      180 * util::kDegreesToRadians, Eigen::Vector3d::UnitX()));
  Transformation glCamera_T_mtCamera =
      Transformation::FromMatrix(affine_glCamera_Q_mtCamera.matrix());

  Transformation mtWorld_T_mtPhysicalCamera =
      Transformation::FromMatrix(glWorld_T_mtWorld.Inverse().ToMatrix4x4() *
                                 glWorld_T_glPhysicalCamera.ToMatrix4x4() *
                                 glCamera_T_mtCamera.ToMatrix4x4());

  motion_emitter_->OnCameraPose(timestamp_us, mtWorld_T_mtPhysicalCamera);
}

void UnrealPluginImpl::OnImage(uint64_t timestamp_us, uint16_t width,
                               uint16_t height, void* data) {
  // Convert the BGRA8 image from Unreal to RGBA8 format. Also rotate the image
  // 180 degrees so it matches the image we normally get from Android device in
  // reverse landscape orientation.
  cv::Mat bgra_mat = cv::Mat(height, width, CV_8UC4, data);
  cv::Mat rgb_mat = cv::Mat(height, width, CV_8UC3);
  cv::cvtColor(bgra_mat, rgb_mat, cv::COLOR_BGRA2RGB);
  cv::rotate(rgb_mat, rgb_mat, util::kReverseLandscapeCvRotateCode);

  util::ImageMetadata metadata;
  metadata.timestamp_ns = timestamp_us * 1000;

  auto image = std::make_shared<const util::Image>(
      util::ImageFormat::kRGB, width, height,
      std::make_unique<cv::Mat>(std::move(rgb_mat)), metadata);
  data_emitter_->OnImage(image);
}

int32_t UnrealPluginImpl::OnGenerateAudio(int16_t* out_buffer,
                                          int32_t num_samples) {
  return audio_output_stream_->OnGenerateAudio(out_buffer, num_samples);
}

void UnrealPluginImpl::SetControlSignalListener(
    UnrealControlSignalListener* listener) {
  absl::MutexLock lock(&mutex_);
  control_signal_listener_ = listener;
}

void UnrealPluginImpl::SetDebugLogListener(UnrealDebugLogListener* listener) {
  absl::MutexLock lock(&mutex_);
  debug_log_listener_ = listener;
}

void UnrealPluginImpl::SetLogEventCallback(
    const logging::LogEventCallback& callback) {
  logger_->SetLogEventCallback(callback);
}

void UnrealPluginImpl::OnControlSignal(
    const environment::ControlSignal& control_signal) {
  absl::MutexLock lock(&mutex_);
  if (control_signal_listener_) {
    UnrealControlSignal unreal_signal;
    unreal_signal.stop = control_signal.stop;
    unreal_signal.lateral_movement_meters =
        control_signal.lateral_movement_meters;
    unreal_signal.rotation_movement_degrees =
        control_signal.rotation_movement_degrees;
    unreal_signal.turn_angle_degrees = control_signal.turn_angle_degrees;
    unreal_signal.turn_point_distance_meters =
        control_signal.turn_point_distance_meters;
    if (control_signal.turn_point.has_value()) {
      unreal_signal.turn_point.x = control_signal.turn_point->x();
      unreal_signal.turn_point.y = control_signal.turn_point->y();
    }
    last_control_signal_ = unreal_signal;
    control_signal_listener_->OnControlSignal(unreal_signal);
  }
}

void UnrealPluginImpl::OnDebugMessage(DebugLogLevel level,
                                      const std::string& message) {
  absl::MutexLock lock(&mutex_);
  if (debug_log_listener_) {
    debug_log_listener_->OnDebugLog(message.c_str());
  }
}

GuidelineRunnerPose UnrealPluginImpl::GetRunnerPose() {
  GuidelineRunnerPose result;
  auto pose = engine_->guidance_system().CurrentPositionAndDirection();
  if (pose.ok()) {
    result.position.x = pose->p().x();
    result.position.y = pose->p().y();

    auto forward_vector =
        util::ComputeForwardDirectionFromQuaternion(pose->q());
    forward_vector.z() = 0;
    forward_vector.normalize();

    result.direction.x = forward_vector.x();
    result.direction.y = forward_vector.y();
  }
  return result;
}

void UnrealPluginImpl::GetGuidelinePoints(GuidelinePoints* out) {
  CHECK(out->points == nullptr) << "GuidelinePoints must be released first";

  auto guideline = engine_->guidance_system().GetGuideline();
  if (!guideline.ok()) {
    out->points = nullptr;
    out->num_points = 0;
    return;
  }

  out->points = new GuidelineVector2[guideline->size()];
  out->num_points = guideline->size();
  for (int i = 0; i < guideline->size(); ++i) {
    auto p = guideline->at(i);
    out->points[i].x = p.x();
    out->points[i].y = p.y();
  }
}

void UnrealPluginImpl::ReleaseGuidelinePoints(GuidelinePoints* points) {
  if (points->points != nullptr) {
    delete[] points->points;
    points->points = nullptr;
    points->num_points = 0;
  }
}

UnrealControlSignal UnrealPluginImpl::GetLastControlSignal() {
  absl::MutexLock lock(&mutex_);
  return last_control_signal_;
}

void UnrealPluginImpl::GetCameraImageDimensions(float& width, float& height) {
  width = camera_model_->image_width();
  height = camera_model_->image_height();
}

void UnrealPluginImpl::GetProjectionMatrix(float near, float far,
                                           float* out_col_major_4x4) {
  Eigen::Matrix4d projection;
  camera::GlCameraProjectionMatrix(*camera_model_, near, far, projection);
  Eigen::Map<Eigen::Matrix4f> map(out_col_major_4x4);
  map = projection.cast<float>();
}

GuidelineEngine& UnrealPluginImpl::guideline_engine() { return *engine_; }

}  // namespace guideline::unreal
