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

#ifndef PROJECT_GUIDELINE_ENGINE_GUIDELINE_ENGINE_H_
#define PROJECT_GUIDELINE_ENGINE_GUIDELINE_ENGINE_H_

#include <atomic>
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "project_guideline/audio/audio_output_stream.h"
#include "project_guideline/audio/audio_system.h"
#include "project_guideline/data/data_source.h"
#include "project_guideline/environment/guidance_system.h"
#include "project_guideline/logging/guideline_logger.h"
#include "project_guideline/motion/motion_tracker.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/vision/guideline_detector.h"

namespace guideline::engine {

class GuidelineEngine {
 public:
  static absl::StatusOr<std::unique_ptr<GuidelineEngine>> Create(
      const GuidelineEngineConfig& config,
      std::shared_ptr<data::DataSource> data_source,
      std::shared_ptr<motion::MotionTracker> motion_tracker,
      std::shared_ptr<audio::AudioOutputStream> audio_output_stream,
      std::shared_ptr<logging::GuidelineLogger> logger);

  ~GuidelineEngine();

  absl::Status Start();
  absl::Status Stop();

  void OnBatteryLevel(int level);

  vision::GuidelineDetector& guideline_detector() {
    return *guideline_detector_;
  }

  data::DataSource& data_source() { return *data_source_; }

  environment::GuidanceSystem& guidance_system() { return *guidance_system_; }

  motion::MotionTracker& motion_tracker() { return *motion_tracker_; }

 private:
  GuidelineEngine(std::shared_ptr<data::DataSource> data_source,
                  std::shared_ptr<motion::MotionTracker> motion_tracker,
                  std::unique_ptr<audio::AudioSystem> audio_system,
                  std::unique_ptr<vision::GuidelineDetector> guideline_detector,
                  std::unique_ptr<environment::GuidanceSystem> guidance_system,
                  std::shared_ptr<logging::GuidelineLogger> logger);

  absl::Status Initialize();

  std::atomic<bool> started_ = false;
  std::shared_ptr<data::DataSource> data_source_;
  std::shared_ptr<motion::MotionTracker> motion_tracker_;
  std::unique_ptr<audio::AudioSystem> audio_system_;
  std::unique_ptr<vision::GuidelineDetector> guideline_detector_;
  std::unique_ptr<environment::GuidanceSystem> guidance_system_;
  std::shared_ptr<logging::GuidelineLogger> logger_;
};

}  // namespace guideline::engine

#endif  // PROJECT_GUIDELINE_ENGINE_GUIDELINE_ENGINE_H_
