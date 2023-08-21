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

#include "project_guideline/engine/guideline_engine.h"

#include "absl/functional/bind_front.h"
#include "absl/status/status.h"
#include "project_guideline/environment/environment.h"
#include "project_guideline/environment/guideline_aggregator.h"
#include "project_guideline/environment/human_representation.h"
#include "project_guideline/environment/occupancy_map.h"
#include "project_guideline/environment/path_planning.h"
#include "project_guideline/environment/point_cloud.h"
#include "project_guideline/util/status.h"

namespace guideline::engine {

namespace {
using audio::AudioSystem;
using data::DataSource;
using environment::CameraPoseBasedHuman;
using environment::Environment;
using environment::GetGuidelineAggregator;
using environment::GuidanceSystem;
using environment::OccupancyMap;
using environment::PointCloud;
using environment::SimpleControlSystem;
using logging::GuidelineLogger;
using motion::MotionTracker;
using vision::GuidelineDetector;
}  // namespace

absl::StatusOr<std::unique_ptr<GuidelineEngine>> GuidelineEngine::Create(
    const GuidelineEngineConfig& config,
    std::shared_ptr<DataSource> data_source,
    std::shared_ptr<MotionTracker> motion_tracker,
    std::shared_ptr<audio::AudioOutputStream> audio_output_stream,
    std::shared_ptr<logging::GuidelineLogger> logger) {
  GL_ASSIGN_OR_RETURN(auto guideline_detector,
                      GuidelineDetector::Create(config.detector_options()));

  GL_ASSIGN_OR_RETURN(auto human, CameraPoseBasedHuman::Create(
                                      config.human_representation_options()));

  GL_ASSIGN_OR_RETURN(auto audio_system,
                      AudioSystem::Create(config.audio_system_options(), logger,
                                          audio_output_stream));
  GL_ASSIGN_OR_RETURN(
      auto guideline_aggregator,
      GetGuidelineAggregator(config.guideline_aggregator_options()));

  auto point_cloud = PointCloud::Create(config.point_cloud_options());
  auto occupancy_map = OccupancyMap::Create(config.occupancy_map_options());
  ASSIGN_OR_RETURN(
      auto environment,
      Environment::Create(std::move(human), std::move(guideline_aggregator),
                          std::move(point_cloud), std::move(occupancy_map),
                          nullptr));

  GL_ASSIGN_OR_RETURN(
      auto control_system,
      SimpleControlSystem::Create(config.control_system_options()));

  GL_ASSIGN_OR_RETURN(auto guidance_system,
                      GuidanceSystem::Create(config.guidance_system_options(),
                                             logger, std::move(environment),
                                             std::move(control_system)));

  std::unique_ptr<GuidelineEngine> engine =
      absl::WrapUnique(new GuidelineEngine(
          std::move(data_source), std::move(motion_tracker),
          std::move(audio_system), std::move(guideline_detector),
          std::move(guidance_system), std::move(logger)));
  GL_RETURN_IF_ERROR(engine->Initialize());
  return engine;
}

GuidelineEngine::GuidelineEngine(
    std::shared_ptr<data::DataSource> data_source,
    std::shared_ptr<motion::MotionTracker> motion_tracker,
    std::unique_ptr<audio::AudioSystem> audio_system,
    std::unique_ptr<vision::GuidelineDetector> guideline_detector,
    std::unique_ptr<environment::GuidanceSystem> guidance_system,
    std::shared_ptr<logging::GuidelineLogger> logger)
    : data_source_(std::move(data_source)),
      motion_tracker_(std::move(motion_tracker)),
      audio_system_(std::move(audio_system)),
      guideline_detector_(std::move(guideline_detector)),
      guidance_system_(std::move(guidance_system)),
      logger_(std::move(logger)) {}

GuidelineEngine::~GuidelineEngine() {
  CHECK_OK(Stop()) << "Failed to stop GuidelineEngine";
}

absl::Status GuidelineEngine::Initialize() {
  // GuidelineDetector uses the image callbacks.
  CHECK_OK(data_source_->AddImageCallback(absl::bind_front(
      &GuidelineDetector::OnImage, guideline_detector_.get())));

  // Set up the GuidanceSystem callbacks.
  motion_tracker_->AddCameraMotionCallback(
      absl::bind_front(&GuidanceSystem::OnCameraPose, guidance_system_.get()));
  motion_tracker_->AddTrackingFeaturesCallback(absl::bind_front(
      &GuidanceSystem::OnTrackingFeatures, guidance_system_.get()));
  motion_tracker_->AddTrackingStateCallback(absl::bind_front(
      &GuidanceSystem::OnTrackingStateChanged, guidance_system_.get()));
  guideline_detector_->AddCallback(
      absl::bind_front(&GuidanceSystem::OnDetection, guidance_system_.get()));

  // AudioSystem responds to the control signal callbacks.
  guidance_system_->AddControlSignalCallback(
      absl::bind_front(&AudioSystem::OnControlSignal, audio_system_.get()));

  return absl::OkStatus();
}

absl::Status GuidelineEngine::Start() {
  CHECK(!started_) << "Already started";
  GL_RETURN_IF_ERROR(audio_system_->Start());
  GL_RETURN_IF_ERROR(motion_tracker_->Start());
  GL_RETURN_IF_ERROR(guideline_detector_->Start());
  GL_RETURN_IF_ERROR(data_source_->Start());
  started_ = true;

  return absl::OkStatus();
}

absl::Status GuidelineEngine::Stop() {
  if (!started_) {
    return absl::OkStatus();
  }
  GL_RETURN_IF_ERROR(data_source_->Stop());
  GL_RETURN_IF_ERROR(motion_tracker_->Stop());
  GL_RETURN_IF_ERROR(guideline_detector_->Stop());
  GL_RETURN_IF_ERROR(guidance_system_->Stop());
  GL_RETURN_IF_ERROR(audio_system_->Stop());
  started_ = false;

  return absl::OkStatus();
}

void GuidelineEngine::OnBatteryLevel(int level) {
  audio_system_->OnBatteryLevel(level);
}

}  // namespace guideline::engine
