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

#include "project_guideline/audio/demo_resonant_sound_pack.h"

#include <algorithm>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "project_guideline/audio/assets/midleft_embed.h"
#include "project_guideline/audio/assets/midright_embed.h"
#include "project_guideline/audio/assets/obstacle_embed.h"
#include "project_guideline/audio/assets/sharpleft_embed.h"
#include "project_guideline/audio/assets/sharpright_embed.h"
#include "project_guideline/audio/assets/slightleft_embed.h"
#include "project_guideline/audio/assets/slightright_embed.h"
#include "project_guideline/audio/assets/steering_off_course_impulse_tick_embed.h"
#include "project_guideline/audio/assets/steering_on_course_impulse_tick_embed.h"
#include "project_guideline/audio/assets/straight_embed.h"
#include "project_guideline/audio/assets/warning_pitch_shift_2x_embed.h"
#include "project_guideline/util/status.h"

namespace guideline::audio {

absl::StatusOr<std::unique_ptr<DemoResonantSoundPack>>
DemoResonantSoundPack::Create(
    SoundPlayer::Factory sound_player_factory,
    std::shared_ptr<logging::GuidelineLogger> logger) {
  return absl::WrapUnique(
      new DemoResonantSoundPack(sound_player_factory(), std::move(logger)));
}

absl::Status DemoResonantSoundPack::Initialize() {
  GL_ASSIGN_OR_RETURN(slight_left_sound_, sound_player_->LoadStereoSound(
                                              slightleft_embed_create()));
  GL_ASSIGN_OR_RETURN(slight_right_sound_, sound_player_->LoadStereoSound(
                                               slightright_embed_create()));
  GL_ASSIGN_OR_RETURN(mid_left_sound_,
                      sound_player_->LoadStereoSound(midleft_embed_create()));
  GL_ASSIGN_OR_RETURN(mid_right_sound_,
                      sound_player_->LoadStereoSound(midright_embed_create()));
  GL_ASSIGN_OR_RETURN(sharp_left_sound_,
                      sound_player_->LoadStereoSound(sharpleft_embed_create()));
  GL_ASSIGN_OR_RETURN(sharp_right_sound_, sound_player_->LoadStereoSound(
                                              sharpright_embed_create()));
  GL_ASSIGN_OR_RETURN(straight_sound_,
                      sound_player_->LoadStereoSound(straight_embed_create()));

  GL_ASSIGN_OR_RETURN(steering_off_course_sound_,
                      sound_player_->LoadResonantSound(
                          steering_off_course_impulse_tick_embed_create()));
  sound_player_->SetLoop(steering_off_course_sound_, true);

  GL_ASSIGN_OR_RETURN(steering_on_course_sound_,
                      sound_player_->LoadResonantSound(
                          steering_on_course_impulse_tick_embed_create()));
  sound_player_->SetLoop(steering_on_course_sound_, true);

  GL_ASSIGN_OR_RETURN(
      warning_sound_,
      sound_player_->LoadResonantSound(warning_pitch_shift_2x_embed_create()));
  sound_player_->SetLoop(warning_sound_, true);

  GL_ASSIGN_OR_RETURN(obstacle_sound_,
                      sound_player_->LoadStereoSound(obstacle_embed_create()));
  sound_player_->SetLoop(obstacle_sound_, true);

  return absl::OkStatus();
}

void DemoResonantSoundPack::OnControlSignal(
    const environment::ControlSignal& signal) {
  if (signal.obstacle_ahead) {
    if (!sound_player_->IsPlaying(obstacle_sound_)) {
      sound_player_->StopAll();
      sound_player_->Play(obstacle_sound_);
    }
    return;
  } else if (signal.obstacle_only_mode) {
    sound_player_->StopAll();
    return;
  } else {
    sound_player_->Stop(obstacle_sound_);
  }

  // The steering sound will appear to come from the direction of the line
  // relative to the user (user moves towards sound).
  sound_player_->Play(steering_on_course_sound_);
  sound_player_->SetSourceVolume(steering_on_course_sound_, 1.);

  // TODO(b/221399309): Tweak these values.
  const float kLateralMovementRangeMeters = 1.5f;
  const float kWarningRangeMeters = 3.0f;
  float line_position =
      std::clamp(signal.lateral_movement_meters / kLateralMovementRangeMeters,
                 -1.0f, 1.0f);
  sound_player_->SetSourcePosition(steering_on_course_sound_, line_position, 0.,
                                   0.);

  // Warning sound will play opposite of the line (user moves away from sound).
  if (std::abs(signal.lateral_movement_meters) > kLateralMovementRangeMeters) {
    float warning_volume =
        std::clamp((std::abs(signal.lateral_movement_meters) -
                    kLateralMovementRangeMeters) /
                       kWarningRangeMeters,
                   0.f, 1.f);

    const float kWarningPositionMeters = 3.0f;
    float warning_position =
        kWarningPositionMeters * (signal.lateral_movement_meters > 0 ? -1 : 1);
    sound_player_->SetSourcePosition(warning_sound_, warning_position, 0., 0.);
    sound_player_->SetSourceVolume(warning_sound_, warning_volume);
    sound_player_->Play(warning_sound_);
  } else {
    sound_player_->Stop(warning_sound_);
  }

  // If we have rotational movement, change the speed/pitch of the steering
  // sound to indicate rotation. Faster means rotate right, slower means rotate
  // left.
  if (std::abs(signal.rotation_movement_degrees) > 0) {
    float rotation =
        std::clamp(std::abs(signal.rotation_movement_degrees) / 90.f, 0.f, 1.f);
    const float kMaxPitchChange = 2.f;
    float pitch = 1.f + rotation * (kMaxPitchChange - 1);
    float playback_rate =
        signal.rotation_movement_degrees < 0 ? pitch : 1.f / pitch;
    sound_player_->SetPlaybackRate(steering_on_course_sound_, playback_rate);
  }

  // If there is an upcoming turn then an a sound will play to alert the user
  // of a slight, mid, sharp left or right. The volume will become louder as the
  // turn approaches.
  static const float kMinTurnAngleDegrees = 10;
  static const float kSlightTurnAngleDegrees = 25;
  static const float kMidTurnAngleDegrees = 45;

  float abs_turn_angle_degrees = std::abs(signal.turn_angle_degrees);
  if (abs_turn_angle_degrees > kMinTurnAngleDegrees &&
      !sound_player_->IsPlayingExclusiveSound()) {
    bool turn_left = signal.turn_angle_degrees < 0;
    SoundId turn_sound_id;
    if (abs_turn_angle_degrees < kSlightTurnAngleDegrees) {
      turn_sound_id = turn_left ? slight_left_sound_ : slight_right_sound_;
    } else if (abs_turn_angle_degrees < kMidTurnAngleDegrees) {
      turn_sound_id = turn_left ? mid_left_sound_ : mid_right_sound_;
    } else {
      turn_sound_id = turn_left ? sharp_left_sound_ : sharp_right_sound_;
    }

    const float kTurnPointMinVolumeDistanceMeters = 15.;
    float turn_volume = std::clamp((kTurnPointMinVolumeDistanceMeters -
                                    signal.turn_point_distance_meters) /
                                       kTurnPointMinVolumeDistanceMeters,
                                   0.3f, 0.8f);
    sound_player_->SetSourceVolume(turn_sound_id, turn_volume);
    sound_player_->PlayExclusiveSoundOnce(turn_sound_id);
  }
}

}  // namespace guideline::audio
