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

#include "project_guideline/audio/sound_player.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/synchronization/mutex.h"
#include "project_guideline/util/status.h"

namespace guideline::audio {

absl::StatusOr<SoundId> SoundPlayer::LoadResonantSound(
    const util::EmbeddedFileToc* toc) {
  vraudio::ResonanceAudioApi::SourceId source_id =
      resonance_audio_->CreateSoundObjectSource(
          vraudio::RenderingMode::kBinauralMediumQuality);
  resonance_audio_->SetSourcePosition(source_id, 0, 0, 0);
  return LoadSound(source_id, toc);
}

absl::StatusOr<SoundId> SoundPlayer::LoadStereoSound(
    const util::EmbeddedFileToc* toc) {
  vraudio::ResonanceAudioApi::SourceId source_id =
      resonance_audio_->CreateStereoSource(/*num_channels=*/2);
  return LoadSound(source_id, toc, /*mono_to_stereo=*/true);
}

absl::StatusOr<SoundId> SoundPlayer::LoadSound(
    const vraudio::ResonanceAudioApi::SourceId source_id,
    const util::EmbeddedFileToc* toc, bool mono_to_stereo) {
  absl::MutexLock lock(&mutex_);
  auto sound_data = std::make_unique<std::string>(toc[0].data, toc[0].size);
  GL_ASSIGN_OR_RETURN(
      auto renderer,
      AudioStreamRenderer::Create(sample_rate_hz_, frames_per_buffer_,
                                  std::move(sound_data), mono_to_stereo));
  auto sound_id = ++next_sound_id_;
  loaded_sounds_.emplace(
      sound_id, std::make_unique<LoadedSound>(source_id, std::move(renderer)));
  return sound_id;
}

void SoundPlayer::Play(SoundId sound_id) {
  absl::MutexLock lock(&mutex_);
  auto& sound = *loaded_sounds_.at(sound_id);
  sound.renderer->Play();
}

bool SoundPlayer::IsPlaying(SoundId sound_id) {
  absl::MutexLock lock(&mutex_);
  auto& sound = *loaded_sounds_.at(sound_id);
  return sound.renderer->IsPlaying();
}

void SoundPlayer::PlayExclusiveSoundOnce(
    SoundId sound_id, const SoundFinishedCallback& callback) {
  absl::MutexLock lock(&mutex_);
  if (exclusive_sound_.has_value()) {
    auto& stop_sound = *loaded_sounds_.at(exclusive_sound_.value().first);
    stop_sound.renderer->Reset();
  }

  auto& sound = *loaded_sounds_.at(sound_id);
  sound.renderer->Reset();
  sound.renderer->SetLoop(false);
  sound.renderer->Play();
  exclusive_sound_.emplace(sound_id, callback);
}

bool SoundPlayer::IsPlayingExclusiveSound() {
  absl::MutexLock lock(&mutex_);
  return exclusive_sound_.has_value();
}

void SoundPlayer::Reset(SoundId sound_id) {
  absl::MutexLock lock(&mutex_);
  auto& sound = *loaded_sounds_.at(sound_id);
  sound.renderer->Reset();
}

void SoundPlayer::Stop(SoundId sound_id) {
  absl::MutexLock lock(&mutex_);
  auto& sound = *loaded_sounds_.at(sound_id);
  sound.renderer->Stop();
}

void SoundPlayer::SetLoop(SoundId sound_id, bool loop) {
  absl::MutexLock lock(&mutex_);
  auto& sound = *loaded_sounds_.at(sound_id);
  sound.renderer->SetLoop(loop);
}

void SoundPlayer::SetLoopRepeatDelay(SoundId sound_id, absl::Duration delay) {
  absl::MutexLock lock(&mutex_);
  auto& sound = *loaded_sounds_.at(sound_id);
  sound.renderer->SetLoopRepeatDelay(delay);
}

int SoundPlayer::GetCurrentLoopCount(SoundId sound_id) {
  absl::MutexLock lock(&mutex_);
  auto& sound = *loaded_sounds_.at(sound_id);
  return sound.renderer->GetCurrentLoopCount();
}

void SoundPlayer::SetSourceVolume(SoundId sound_id, float volume) {
  absl::MutexLock lock(&mutex_);
  auto& sound = *loaded_sounds_.at(sound_id);
  resonance_audio_->SetSourceVolume(sound.source_id, volume);
}

void SoundPlayer::SetStereoVolume(SoundId sound_id, float left_volume,
                                  float right_volume) {
  absl::MutexLock lock(&mutex_);
  auto& sound = *loaded_sounds_.at(sound_id);
  sound.renderer->SetStereoVolume(left_volume, right_volume);
}

void SoundPlayer::SetSourcePosition(SoundId sound_id, float x, float y,
                                    float z) {
  absl::MutexLock lock(&mutex_);
  auto& sound = *loaded_sounds_.at(sound_id);
  resonance_audio_->SetSourcePosition(sound.source_id, x, y, z);
}

void SoundPlayer::SetPlaybackRate(SoundId sound_id, float playback_rate) {
  absl::MutexLock lock(&mutex_);
  auto& sound = *loaded_sounds_.at(sound_id);
  sound.renderer->SetPlaybackRate(playback_rate);
}

void SoundPlayer::StopAll() {
  absl::MutexLock lock(&mutex_);
  for (auto it = loaded_sounds_.cbegin(); it != loaded_sounds_.cend(); ++it) {
    auto& sound = *it->second;
    sound.renderer->Reset();
  }
}

void SoundPlayer::OnMoreData(size_t num_frames) {
  SoundFinishedCallback exclusive_finished_callback = nullptr;
  SoundId exclusive_sound_id;
  {
    absl::MutexLock lock(&mutex_);

    for (auto it = loaded_sounds_.cbegin(); it != loaded_sounds_.cend(); ++it) {
      auto& sound = *it->second;
      if (!sound.renderer->IsPlaying()) {
        continue;
      }

      resonance_audio_->SetPlanarBuffer(
          sound.source_id, sound.renderer->GetNextBuffer(),
          sound.renderer->GetNumChannels(), num_frames);
    }

    if (exclusive_sound_.has_value()) {
      exclusive_sound_id = exclusive_sound_.value().first;
      auto& sound = *loaded_sounds_.at(exclusive_sound_id);
      if (!sound.renderer->IsPlaying()) {
        exclusive_finished_callback = exclusive_sound_.value().second;
        exclusive_sound_.reset();
      }
    }
  }
  if (exclusive_finished_callback) {
    exclusive_finished_callback(exclusive_sound_id);
  }
}

}  // namespace guideline::audio
