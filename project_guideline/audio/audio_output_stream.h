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

#ifndef PROJECT_GUIDELINE_AUDIO_AUDIO_OUTPUT_STREAM_H_
#define PROJECT_GUIDELINE_AUDIO_AUDIO_OUTPUT_STREAM_H_

#include <cstdint>
#include <functional>

#include "absl/status/status.h"

namespace guideline::audio {

constexpr size_t kDefaultNumChannels = 2;

// Interface for an audio output stream which will typically wrap a platform
// audio API. Based on the sample rate and buffer sizes, the Callback function
// will be invoked at a periodic rate to fetch audio data to play through the
// audio device.
class AudioOutputStream {
 public:
  AudioOutputStream(int sample_rate_hz, size_t frames_per_buffer,
                    size_t num_channels)
      : sample_rate_hz_(sample_rate_hz),
        frames_per_buffer_(frames_per_buffer),
        num_channels_(num_channels) {}

  using Callback = std::function<bool(int16_t* buffer_ptr, size_t num_channels,
                                      size_t num_frames)>;

  virtual ~AudioOutputStream() = default;

  virtual absl::Status StartPlayback(
      const AudioOutputStream::Callback& callback) = 0;

  virtual absl::Status StopPlayback() = 0;

  int GetSampleRateHz() const { return sample_rate_hz_; }

  size_t GetFramesPerBuffer() const { return frames_per_buffer_; }

  size_t GetNumChannels() const { return num_channels_; }

 private:
  const int sample_rate_hz_;
  const size_t frames_per_buffer_;
  const size_t num_channels_;
};

}  // namespace guideline::audio

#endif  // PROJECT_GUIDELINE_AUDIO_AUDIO_OUTPUT_STREAM_H_
