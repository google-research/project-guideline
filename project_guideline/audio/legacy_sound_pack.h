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

#ifndef PROJECT_GUIDELINE_AUDIO_LEGACY_SOUND_PACK_H_
#define PROJECT_GUIDELINE_AUDIO_LEGACY_SOUND_PACK_H_

#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "project_guideline/audio/sound_pack.h"
#include "project_guideline/audio/sound_player.h"
#include "project_guideline/environment/control_signal.h"
#include "project_guideline/logging/guideline_logger.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/embedded_file_toc.h"

namespace guideline::audio {

using PanningStrategy =
    std::function<void(float input, float& left, float& right)>;

using RateStrategy = std::function<float(float input)>;

class LegacySoundPack : public SoundPack {
 public:
  static absl::StatusOr<std::unique_ptr<LegacySoundPack>> Create(
      SoundPlayer::Factory sound_player_factory,
      std::shared_ptr<logging::GuidelineLogger> logger,
      const LegacySoundPackOptions& options =
          LegacySoundPackOptions::default_instance());

  absl::Status Initialize() override;
  void OnControlSignal(const environment::ControlSignal& signal) override;

  std::optional<const util::EmbeddedFileToc*> GetStopSoundOverride() override;

 private:
  explicit LegacySoundPack(std::unique_ptr<SoundPlayer> sound_player,
                           std::shared_ptr<logging::GuidelineLogger> logger,
                           const LegacySoundPackOptions& options)
      : SoundPack(std::move(sound_player), std::move(logger)),
        options_(options) {}

  const LegacySoundPackOptions options_;

  std::optional<const util::EmbeddedFileToc*> stop_sound_override_ =
      std::nullopt;

  PanningStrategy steering_panner_ = nullptr;
  PanningStrategy warning_panner_ = nullptr;
  PanningStrategy curve_panner_ = nullptr;
  RateStrategy steering_rate_strategy_ = nullptr;
  RateStrategy warning_rate_strategy_ = nullptr;
  RateStrategy curve_rate_strategy_ = nullptr;

  SoundId steering_sound_;
  SoundId warning_sound_;
  SoundId curve_sound_;
  SoundId obstacle_sound_;
};

}  // namespace guideline::audio

#endif  // PROJECT_GUIDELINE_AUDIO_LEGACY_SOUND_PACK_H_
