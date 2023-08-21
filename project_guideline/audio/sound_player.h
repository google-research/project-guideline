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

#ifndef PROJECT_GUIDELINE_AUDIO_SOUND_PLAYER_H_
#define PROJECT_GUIDELINE_AUDIO_SOUND_PLAYER_H_

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "project_guideline/audio/audio_stream_renderer.h"
#include "project_guideline/util/embedded_file_toc.h"
#include "resonance_audio_api.h"

namespace guideline::audio {

typedef int SoundId;

class SoundPlayer {
 public:
  using Factory = std::function<std::unique_ptr<SoundPlayer>()>;

  SoundPlayer(std::shared_ptr<vraudio::ResonanceAudioApi> resonance_audio,
              int sample_rate_hz, size_t frames_per_buffer)
      : resonance_audio_(std::move(resonance_audio)),
        sample_rate_hz_(sample_rate_hz),
        frames_per_buffer_(frames_per_buffer) {}

  void OnMoreData(size_t num_frames);

  absl::StatusOr<SoundId> LoadResonantSound(const util::EmbeddedFileToc* toc);

  absl::StatusOr<SoundId> LoadStereoSound(const util::EmbeddedFileToc* toc);

  void Play(SoundId sound_id);

  // Plays a sound on the 'exclusive' channel. Only a single sound can be
  // played at a time on this channel. If another exclusive sound is already
  // playing it will be stopped first.
  using SoundFinishedCallback = std::function<void(SoundId sound_id)>;
  void PlayExclusiveSoundOnce(
      SoundId sound_id, const SoundFinishedCallback& callback = [](SoundId) {});
  bool IsPlaying(SoundId sound_id);
  bool IsPlayingExclusiveSound();

  void Reset(SoundId sound_id);
  void Stop(SoundId sound_id);
  void SetLoop(SoundId sound_id, bool loop);
  void SetLoopRepeatDelay(SoundId sound_id, absl::Duration delay);
  int GetCurrentLoopCount(SoundId sound_id);
  void SetSourceVolume(SoundId sound_id, float volume);
  void SetStereoVolume(SoundId sound_id, float left_volume, float right_volume);
  void SetSourcePosition(SoundId sound_id, float x, float y, float z);
  void SetPlaybackRate(SoundId sound_id, float playback_rate);
  void StopAll();

 private:
  struct LoadedSound {
    explicit LoadedSound(const vraudio::ResonanceAudioApi::SourceId source_id,
                         std::unique_ptr<AudioStreamRenderer> renderer)
        : source_id(source_id), renderer(std::move(renderer)) {}

    const vraudio::ResonanceAudioApi::SourceId source_id;
    std::unique_ptr<AudioStreamRenderer> renderer;
  };

  absl::StatusOr<SoundId> LoadSound(
      vraudio::ResonanceAudioApi::SourceId source_id,
      const util::EmbeddedFileToc* toc, bool mono_to_stereo = false);

  std::shared_ptr<vraudio::ResonanceAudioApi> resonance_audio_;
  int sample_rate_hz_;
  size_t frames_per_buffer_;
  absl::Mutex mutex_;
  int next_sound_id_ ABSL_GUARDED_BY(mutex_) = 0;
  std::map<SoundId, const std::unique_ptr<LoadedSound>> ABSL_GUARDED_BY(mutex_)
      loaded_sounds_;
  std::optional<std::pair<SoundId, SoundFinishedCallback>> exclusive_sound_
      ABSL_GUARDED_BY(mutex_) = std::nullopt;
};

}  // namespace guideline::audio

#endif  // PROJECT_GUIDELINE_AUDIO_SOUND_PLAYER_H_
