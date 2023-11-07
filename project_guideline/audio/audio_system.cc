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

#include "project_guideline/audio/audio_system.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/functional/bind_front.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "project_guideline/audio/assets/loading_embed.h"
#include "project_guideline/audio/assets/low_battery_embed.h"
#include "project_guideline/audio/assets/ready_embed.h"
#include "project_guideline/audio/assets/stopstopstop_embed.h"
#include "project_guideline/audio/audio_output_stream.h"
#include "project_guideline/audio/demo_resonant_sound_pack.h"
#include "project_guideline/audio/legacy_sound_pack.h"
#include "project_guideline/audio/sound_pack.h"
#include "project_guideline/audio/sound_player.h"
#include "project_guideline/audio/vorbis_stream_encoder.h"
#include "project_guideline/logging/guideline_logger.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/status.h"
#include "resonance_audio_api.h"

namespace guideline::audio {

absl::StatusOr<std::unique_ptr<AudioSystem>> AudioSystem::Create(
    const AudioSystemOptions& options,
    std::shared_ptr<logging::GuidelineLogger> logger,
    std::shared_ptr<audio::AudioOutputStream> audio_output_stream,
    SoundPack::Factory sound_pack_factory) {
  std::shared_ptr<vraudio::ResonanceAudioApi> resonance_audio =
      absl::WrapUnique(vraudio::CreateResonanceAudioApi(
          audio_output_stream->GetNumChannels(),
          audio_output_stream->GetFramesPerBuffer(),
          audio_output_stream->GetSampleRateHz()));

  SoundPlayer::Factory sound_player_factory = [resonance_audio,
                                               audio_output_stream]() {
    return std::make_unique<SoundPlayer>(
        resonance_audio, audio_output_stream->GetSampleRateHz(),
        audio_output_stream->GetFramesPerBuffer());
  };

  std::unique_ptr<SoundPack> sound_pack;
  if (sound_pack_factory != nullptr) {
    GL_ASSIGN_OR_RETURN(sound_pack,
                        sound_pack_factory(sound_player_factory, logger));
  } else if (options.has_legacy_sound_pack_options()) {
    GL_ASSIGN_OR_RETURN(sound_pack, LegacySoundPack::Create(
                                        sound_player_factory, logger,
                                        options.legacy_sound_pack_options()));
  } else {
    GL_ASSIGN_OR_RETURN(sound_pack, DemoResonantSoundPack::Create(
                                        sound_player_factory, logger));
  }

  auto audio_system = absl::WrapUnique(
      new AudioSystem(options, std::move(resonance_audio),
                      std::move(sound_pack), std::move(audio_output_stream)));
  GL_RETURN_IF_ERROR(audio_system->Initialize());
  return audio_system;
}

AudioSystem::AudioSystem(
    const AudioSystemOptions& options,
    std::shared_ptr<vraudio::ResonanceAudioApi> resonance_audio,
    std::unique_ptr<SoundPack> sound_pack,
    std::shared_ptr<audio::AudioOutputStream> audio_output_stream)
    : options_(options),
      resonance_audio_(resonance_audio),
      sound_pack_(std::move(sound_pack)),
      audio_output_stream_(audio_output_stream),
      base_sound_player_(std::make_unique<SoundPlayer>(
          resonance_audio, audio_output_stream_->GetSampleRateHz(),
          audio_output_stream_->GetFramesPerBuffer())) {}

AudioSystem::~AudioSystem() { CHECK_OK(Stop()); }

absl::Status AudioSystem::Initialize() {
  absl::MutexLock lock(&mutex_);

  GL_RETURN_IF_ERROR(sound_pack_->Initialize());

  GL_ASSIGN_OR_RETURN(initializing_sound_, base_sound_player_->LoadStereoSound(
                                               loading_embed_create()));
  base_sound_player_->SetLoop(initializing_sound_, true);

  GL_ASSIGN_OR_RETURN(
      ready_sound_, base_sound_player_->LoadStereoSound(ready_embed_create()));

  GL_ASSIGN_OR_RETURN(low_battery_sound_, base_sound_player_->LoadStereoSound(
                                              low_battery_embed_create()));

  auto stop_sound_override = sound_pack_->GetStopSoundOverride();
  GL_ASSIGN_OR_RETURN(stop_sound_, base_sound_player_->LoadStereoSound(
                                       stop_sound_override.has_value()
                                           ? *stop_sound_override
                                           : stopstopstop_embed_create()));

  resonance_audio_->SetHeadPosition(0.0f, 0.0f, 0.0f);

  return absl::OkStatus();
}

void AudioSystem::RecordToFile(absl::string_view recording_filename) {
  recording_filename_ = std::string(recording_filename);
}

absl::Status AudioSystem::Start() {
  if (!recording_filename_.empty()) {
    GL_ASSIGN_OR_RETURN(
        stream_encoder_,
        audio::VorbisStreamEncoder::Open(
            recording_filename_, audio_output_stream_->GetNumChannels(),
            audio_output_stream_->GetSampleRateHz(),
            /*quality=*/1.0f));
  }

  GL_RETURN_IF_ERROR(audio_output_stream_->StartPlayback(
      absl::bind_front(&AudioSystem::OnMoreData, this)));

  {
    absl::MutexLock lock(&mutex_);
    started_ = true;

    BeginInitializationState();
  }

  return absl::OkStatus();
}

absl::Status AudioSystem::Stop() {
  {
    absl::MutexLock lock(&mutex_);
    if (!started_) {
      return absl::OkStatus();
    }
    started_ = false;
  }

  GL_RETURN_IF_ERROR(audio_output_stream_->StopPlayback());

  if (stream_encoder_) {
    GL_RETURN_IF_ERROR(stream_encoder_->FlushAndClose());
    stream_encoder_ = nullptr;
  }

  StopSoundPlayer();

  return absl::OkStatus();
}

absl::Status AudioSystem::WaitForAudioStream() {
  if (!audio_stream_started_.WaitForNotificationWithTimeout(absl::Seconds(5))) {
    return absl::InternalError("Audio stream failed to start within timeout");
  }
  return absl::OkStatus();
}

void AudioSystem::EnableRecording() { recording_ = true; }

bool AudioSystem::OnMoreData(int16_t* buffer_ptr, size_t num_channels,
                             size_t num_frames) {
  if (!audio_stream_started_.HasBeenNotified()) {
    audio_stream_started_.Notify();
  }

  absl::MutexLock lock(&mutex_);

  if (!started_) {
    return false;
  }

  base_sound_player_->OnMoreData(num_frames);
  sound_pack_->OnMoreData(num_frames);

  bool result = resonance_audio_->FillInterleavedOutputBuffer(
      num_channels, num_frames, buffer_ptr);
  if (!result) {
    std::fill(buffer_ptr, buffer_ptr + (num_frames * num_channels), 0);
  }

  if (stream_encoder_ && recording_.load()) {
    CHECK_OK(stream_encoder_->AddInterleavedBuffer(buffer_ptr, num_channels,
                                                   num_frames));
  }

  return result;
}

void AudioSystem::OnControlSignal(const environment::ControlSignal& signal) {
  absl::MutexLock lock(&mutex_);

  // If we get a stop signal, play the "Stop" sound immediately.
  if (signal.stop) {
    if (state_ != ControlSoundState::STOPPING &&
        state_ != ControlSoundState::INITIALIZING) {
      if (options_.stop_alert()) {
        state_ = ControlSoundState::STOPPING;
        StopSoundPlayer();
        base_sound_player_->PlayExclusiveSoundOnce(
            stop_sound_,
            absl::bind_front(&AudioSystem::OnExclusiveSoundFinished, this));
      } else {
        BeginInitializationState();
      }
    }
    return;
  }

  if (state_ == ControlSoundState::INITIALIZING) {
    if (options_.quick_initialization()) {
      BeginGuidingState();
      return;
    }

    // Let the initializing sound play through at least once.
    if (base_sound_player_->GetCurrentLoopCount(initializing_sound_) < 1) {
      return;
    }

    // Play the "Ready" sound.
    base_sound_player_->Stop(initializing_sound_);
    base_sound_player_->PlayExclusiveSoundOnce(
        ready_sound_,
        absl::bind_front(&AudioSystem::OnExclusiveSoundFinished, this));
    state_ = ControlSoundState::READY;
    return;
  }

  if (state_ == ControlSoundState::GUIDING) {
    sound_pack_->OnControlSignal(signal);
  }
}

void AudioSystem::OnBatteryLevel(int level) {
  if (level < kLowBatteryWarningThreshold) {
    if (!alerting_low_battery_) {
      base_sound_player_->Reset(low_battery_sound_);
      base_sound_player_->SetLoop(low_battery_sound_, true);
      base_sound_player_->SetLoopRepeatDelay(low_battery_sound_,
                                             kLowBatteryWarningInterval);
      base_sound_player_->Play(low_battery_sound_);
      alerting_low_battery_ = true;
    }
  } else if (alerting_low_battery_) {
    base_sound_player_->Stop(low_battery_sound_);
    alerting_low_battery_ = false;
  }
}

void AudioSystem::BeginInitializationState() {
  StopSoundPlayer();
  base_sound_player_->SetSourceVolume(initializing_sound_, 1.);
  base_sound_player_->Play(initializing_sound_);
  state_ = ControlSoundState::INITIALIZING;
}

void AudioSystem::OnExclusiveSoundFinished(SoundId sound_id) {
  mutex_.AssertHeld();
  if (sound_id == ready_sound_ && state_ == ControlSoundState::READY) {
    BeginGuidingState();
  } else if (sound_id == stop_sound_ && state_ == ControlSoundState::STOPPING) {
    BeginInitializationState();
  }
}

void AudioSystem::BeginGuidingState() {
  StopSoundPlayer();
  state_ = ControlSoundState::GUIDING;
  CHECK_OK(sound_pack_->Start());
}

void AudioSystem::StopSoundPlayer() {
  CHECK_OK(sound_pack_->Stop());
  alerting_low_battery_ = false;
  base_sound_player_->StopAll();
}

}  // namespace guideline::audio
