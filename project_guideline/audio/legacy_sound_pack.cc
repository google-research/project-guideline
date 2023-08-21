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

#include "project_guideline/audio/legacy_sound_pack.h"

#include <complex>
#include <cstdlib>
#include <memory>
#include <optional>
#include <utility>

#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "project_guideline/audio/assets/legacy_steering_20211014_embed.h"
#include "project_guideline/audio/assets/legacy_steering_embed.h"
#include "project_guideline/audio/assets/legacy_v4_1_steering_embed.h"
#include "project_guideline/audio/assets/legacy_v4_1_stop_embed.h"
#include "project_guideline/audio/assets/legacy_v4_1_warning_embed.h"
#include "project_guideline/audio/assets/legacy_v4_2_steering_embed.h"
#include "project_guideline/audio/assets/legacy_v4_2_stop_embed.h"
#include "project_guideline/audio/assets/legacy_v4_2_warning_embed.h"
#include "project_guideline/audio/assets/legacy_warning_embed.h"
#include "project_guideline/audio/assets/obstacle_embed.h"
#include "project_guideline/audio/assets/warning_pitch_shift_2x_embed.h"
#include "project_guideline/audio/sound_player.h"
#include "project_guideline/logging/guideline_logger.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/embedded_file_toc.h"
#include "project_guideline/util/lerp.h"
#include "project_guideline/util/status.h"

namespace guideline::audio {

namespace {

static PanningStrategy LegacyHardPan(const bool reversed) {
  return [reversed](float input, float& left, float& right) {
    float l = util::ClampedLerp<float>(input, -0.1f, 1, 0, 1);
    float r = util::ClampedLerp<float>(input, -1, 0.1f, 1, 0);
    left = reversed ? r : l;
    right = reversed ? l : r;
  };
}

static PanningStrategy LinearThresholdPan(float threshold) {
  return [threshold](float input, float& left, float& right) {
    float volume =
        util::ClampedLerp<float>(std::abs(input), threshold, 1, 0, 1);
    left = input > 0 ? volume : 0;
    right = input < 0 ? volume : 0;
  };
}

float ApplyShaping(float x, float sensitivity_curve) {
  if (sensitivity_curve == 0) {
    return x;
  }
  float mid_y = util::Lerp<float>(sensitivity_curve, 0, 1, 0.5, 0.01);
  if (x < -0.5) {
    return util::Lerp<float>(x, -1, -0.5f, -1, -mid_y);
  } else if (x < 0) {
    return util::Lerp<float>(x, -0.5f, 0, -mid_y, 0);
  } else if (x < 0.5) {
    return util::Lerp<float>(x, 0, 0.5f, 0, mid_y);
  } else {
    return util::Lerp<float>(x, 0.5f, 1, mid_y, 1);
  }
}

}  // namespace

absl::StatusOr<std::unique_ptr<LegacySoundPack>> LegacySoundPack::Create(
    SoundPlayer::Factory sound_player_factory,
    std::shared_ptr<logging::GuidelineLogger> logger,
    const LegacySoundPackOptions& options) {
  return absl::WrapUnique(
      new LegacySoundPack(sound_player_factory(), std::move(logger), options));
}

absl::Status LegacySoundPack::Initialize() {
  const util::EmbeddedFileToc* steering_sound_toc = nullptr;
  const util::EmbeddedFileToc* warning_sound_toc = nullptr;

  switch (options_.type()) {
    case LegacySoundPackOptions::V0:
      steering_sound_toc = legacy_steering_embed_create();
      warning_sound_toc = legacy_v4_1_warning_embed_create();
      steering_panner_ = LegacyHardPan(/*reversed=*/false);
      warning_panner_ = LinearThresholdPan(0.8f);
      steering_rate_strategy_ = [](float input) {
        return util::ClampedLerp<float>(std::abs(input), 0, 1, 1, 2,
                                        &util::SquareLerpInterpolator);
      };
      warning_rate_strategy_ = [](float input) {
        return util::ClampedLerp<float>(std::abs(input), 0.8, 1, 1, 2,
                                        &util::SquareLerpInterpolator);
      };
      break;
    case LegacySoundPackOptions::V4_0:
      steering_sound_toc = legacy_steering_20211014_embed_create();
      warning_sound_toc = warning_pitch_shift_2x_embed_create();
      steering_panner_ = LegacyHardPan(/*reversed=*/true);
      warning_panner_ = LinearThresholdPan(0.4f);
      steering_rate_strategy_ = [](float input) { return 1; };
      warning_rate_strategy_ = [](float input) {
        return util::ClampedLerp<float>(std::abs(input), 0.4, 1, 1, 2,
                                        &util::SquareLerpInterpolator);
      };
      break;
    case LegacySoundPackOptions::V4_1:
      steering_sound_toc = legacy_v4_1_steering_embed_create();
      warning_sound_toc = legacy_v4_1_warning_embed_create();
      stop_sound_override_ = legacy_v4_1_stop_embed_create();
      steering_panner_ = LegacyHardPan(/*reversed=*/true);
      warning_panner_ = LinearThresholdPan(0.4f);
      steering_rate_strategy_ = [](float input) {
        return util::ClampedLerp<float>(std::abs(input), 0, 1, 1, 2,
                                        &util::SquareLerpInterpolator);
      };
      warning_rate_strategy_ = [](float input) {
        return util::ClampedLerp<float>(std::abs(input), 0.4, 1, 1, 2,
                                        &util::SquareLerpInterpolator);
      };
      break;
    case LegacySoundPackOptions::V4_2:
      steering_sound_toc = legacy_v4_2_steering_embed_create();
      warning_sound_toc = legacy_v4_2_warning_embed_create();
      stop_sound_override_ = legacy_v4_2_stop_embed_create();
      steering_panner_ = LegacyHardPan(/*reversed=*/true);
      warning_panner_ = LinearThresholdPan(0.4f);
      steering_rate_strategy_ = [](float input) { return 1; };
      warning_rate_strategy_ = [](float input) {
        return util::ClampedLerp<float>(std::abs(input), 0.4, 1, 1, 2,
                                        &util::SquareLerpInterpolator);
      };
      break;
  }

  CHECK(steering_sound_toc);
  CHECK(warning_sound_toc);

  GL_ASSIGN_OR_RETURN(steering_sound_,
                      sound_player_->LoadStereoSound(steering_sound_toc));
  sound_player_->SetLoop(steering_sound_, true);
  GL_ASSIGN_OR_RETURN(warning_sound_,
                      sound_player_->LoadStereoSound(warning_sound_toc));
  sound_player_->SetLoop(warning_sound_, true);
  GL_ASSIGN_OR_RETURN(obstacle_sound_,
                      sound_player_->LoadStereoSound(obstacle_embed_create()));
  sound_player_->SetLoop(obstacle_sound_, true);

  return absl::OkStatus();
}

void LegacySoundPack::OnControlSignal(
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

  float steering_position = util::ClampedLerp<float>(
      signal.target_rotation_movement_degrees, -options_.max_rotation_degrees(),
      options_.max_rotation_degrees(), -1, 1);
  steering_position =
      ApplyShaping(steering_position, options_.sensitivity_curvature());

  float steering_left_volume = 0;
  float steering_right_volume = 0;
  steering_panner_(steering_position, steering_left_volume,
                   steering_right_volume);
  sound_player_->SetStereoVolume(steering_sound_, steering_left_volume,
                                 steering_right_volume);
  sound_player_->SetPlaybackRate(steering_sound_,
                                 steering_rate_strategy_(steering_position));
  sound_player_->Play(steering_sound_);

  float warning_left_volume = 0;
  float warning_right_volume = 0;
  float warning_position = util::ClampedLerp<float>(
      signal.lateral_movement_meters, -options_.warning_threshold_meters(),
      options_.warning_threshold_meters(), -1, 1);
  warning_position =
      ApplyShaping(warning_position, options_.sensitivity_curvature());

  warning_panner_(warning_position, warning_left_volume, warning_right_volume);
  sound_player_->SetStereoVolume(warning_sound_, warning_left_volume,
                                 warning_right_volume);
  sound_player_->SetPlaybackRate(warning_sound_,
                                 warning_rate_strategy_(warning_position));
  sound_player_->Play(warning_sound_);
}

std::optional<const util::EmbeddedFileToc*>
LegacySoundPack::GetStopSoundOverride() {
  return stop_sound_override_;
}

}  // namespace guideline::audio
