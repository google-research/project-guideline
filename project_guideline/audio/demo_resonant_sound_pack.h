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

// SoundPack that demonstrates the use of resonant sounds along with the new
// control signals for rotational/lateral movement and upcoming turns. This is
// not intended for actual navigational use at this time.

#ifndef PROJECT_GUIDELINE_AUDIO_DEMO_RESONANT_SOUND_PACK_H_
#define PROJECT_GUIDELINE_AUDIO_DEMO_RESONANT_SOUND_PACK_H_

#include <memory>
#include <utility>

#include "absl/status/statusor.h"
#include "project_guideline/audio/sound_pack.h"
#include "project_guideline/environment/control_signal.h"

namespace guideline::audio {

class DemoResonantSoundPack : public SoundPack {
 public:
  static absl::StatusOr<std::unique_ptr<DemoResonantSoundPack>> Create(
      SoundPlayer::Factory sound_player_factory,
      std::shared_ptr<logging::GuidelineLogger> logger);

  absl::Status Initialize() override;
  void OnControlSignal(const environment::ControlSignal& signal) override;

 private:
  explicit DemoResonantSoundPack(
      std::unique_ptr<SoundPlayer> sound_player,
      std::shared_ptr<logging::GuidelineLogger> logger)
      : SoundPack(std::move(sound_player), std::move(logger)) {}

  SoundId slight_left_sound_;
  SoundId slight_right_sound_;
  SoundId mid_left_sound_;
  SoundId mid_right_sound_;
  SoundId sharp_left_sound_;
  SoundId sharp_right_sound_;
  SoundId straight_sound_;
  SoundId steering_on_course_sound_;
  SoundId steering_off_course_sound_;
  SoundId warning_sound_;
  SoundId obstacle_sound_;
};

}  // namespace guideline::audio

#endif  // PROJECT_GUIDELINE_AUDIO_DEMO_RESONANT_SOUND_PACK_H_
