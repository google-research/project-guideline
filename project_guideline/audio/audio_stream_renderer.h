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

// Handles rendering an audio file with support for looping.

#ifndef PROJECT_GUIDELINE_AUDIO_AUDIO_STREAM_RENDERER_H_
#define PROJECT_GUIDELINE_AUDIO_AUDIO_STREAM_RENDERER_H_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "project_guideline/audio/audio_buffer.h"

namespace guideline::audio {

class AudioStreamRenderer {
 public:
  // Creates a renderer from an Ogg Vorbis file loaded to memory.
  static absl::StatusOr<std::unique_ptr<AudioStreamRenderer>> Create(
      int output_sample_rate_hz, int frames_per_buffer,
      std::unique_ptr<std::string> ogg_vorbis_data,
      bool mono_to_stereo = false);

  // Sets whether the audio file should loop.
  void SetLoop(bool loop);

  // Sets a delay between loop repetitions.
  void SetLoopRepeatDelay(absl::Duration delay);

  // Sets playback rate which will cause speed/pitch of the audio to change.
  // Value of 1.0 is normal playback rate.
  void SetPlaybackRate(float playback_rate);

  // Starts playing this audio stream.
  void Play();

  // Stops this audio stream.
  void Stop();

  // Resets to the beginning of the audio file.
  void Reset();

  // Gets the next available audio buffer.
  // Should only be called if IsPlaying() returns true.
  const float** GetNextBuffer();

  // Gets the number of audio channels.
  size_t GetNumChannels();

  // Gets whether this audio stream is currently playing. This is true if
  // Play() has been called and either the audio has not reached the end of the
  // file or this stream is set to loop.
  bool IsPlaying();

  // Returns the number of times the audio has looped since the last reset, if
  // SetLoop(true) has been called.
  int GetCurrentLoopCount();

  // Sets the volume of the left and right audio channels
  void SetStereoVolume(float left_volume, float right_volume);

 private:
  explicit AudioStreamRenderer(
      int output_sample_rate_hz, int frames_per_buffer,
      std::unique_ptr<audio::PlanarAudioBuffer> audio_data,
      bool mono_to_stereo);

  const int output_sample_rate_hz_;
  const int frames_per_buffer_;
  std::unique_ptr<audio::PlanarAudioBuffer> audio_data_;
  audio::PlanarAudioBuffer output_buffer_;
  std::vector<const float*> output_buffer_channel_ptrs_;

  absl::Mutex mutex_;
  bool playing_ ABSL_GUARDED_BY(mutex_) = false;
  bool loop_ ABSL_GUARDED_BY(mutex_) = false;
  absl::Duration loop_repeat_delay_ ABSL_GUARDED_BY(mutex_) =
      absl::ZeroDuration();
  absl::Duration remaining_loop_delay_ ABSL_GUARDED_BY(mutex_) =
      absl::ZeroDuration();
  int current_loop_count_ ABSL_GUARDED_BY(mutex_) = 0;
  float playback_rate_ ABSL_GUARDED_BY(mutex_) = 1.;
  bool has_playback_rate_ ABSL_GUARDED_BY(mutex_) = false;
  bool end_of_stream_ ABSL_GUARDED_BY(mutex_) = false;
  size_t data_position_ ABSL_GUARDED_BY(mutex_) = 0;

  float left_volume_ ABSL_GUARDED_BY(mutex_) = 1.;
  float right_volume_ ABSL_GUARDED_BY(mutex_) = 1.;
};

}  // namespace guideline::audio

#endif  // PROJECT_GUIDELINE_AUDIO_AUDIO_STREAM_RENDERER_H_
