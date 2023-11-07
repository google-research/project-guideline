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

#ifndef PROJECT_GUIDELINE_AUDIO_SOUND_PACK_H_
#define PROJECT_GUIDELINE_AUDIO_SOUND_PACK_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "project_guideline/audio/sound_player.h"
#include "project_guideline/environment/control_signal.h"
#include "project_guideline/logging/guideline_logger.h"
#include "project_guideline/util/embedded_file_toc.h"

namespace guideline::audio {

class SoundPack {
 public:
  using Factory = std::function<absl::StatusOr<std::unique_ptr<SoundPack>>(
      SoundPlayer::Factory sound_player_factory,
      std::shared_ptr<logging::GuidelineLogger> logger)>;

  explicit SoundPack(std::unique_ptr<SoundPlayer> sound_player,
                     std::shared_ptr<logging::GuidelineLogger> logger)
      : sound_player_(std::move(sound_player)), logger_(std::move(logger)) {}

  virtual ~SoundPack() = default;

  virtual absl::Status Initialize() = 0;

  virtual absl::Status Start();
  virtual absl::Status Stop();

  virtual void OnControlSignal(const environment::ControlSignal& signal) = 0;

  virtual std::optional<const util::EmbeddedFileToc*> GetStopSoundOverride() {
    return std::nullopt;
  }

  void OnMoreData(size_t num_frames);

 protected:
  std::unique_ptr<SoundPlayer> sound_player_;
  std::shared_ptr<logging::GuidelineLogger> logger_;
};

}  // namespace guideline::audio

#endif  // PROJECT_GUIDELINE_AUDIO_SOUND_PACK_H_
