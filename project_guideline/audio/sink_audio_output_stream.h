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

#ifndef PROJECT_GUIDELINE_AUDIO_SINK_AUDIO_OUTPUT_STREAM_H_
#define PROJECT_GUIDELINE_AUDIO_SINK_AUDIO_OUTPUT_STREAM_H_

#include <cstdint>
#include <memory>
#include <thread>  // NOLINT(build/c++11)
#include <vector>

#include "project_guideline/audio/audio_output_stream.h"
#include "project_guideline/util/clock.h"

namespace guideline::audio {

// Implementation of AudioOutputStream that simulates an output device by
// invoking the callback at a regular rate in real time, but does nothing with
// the audio data.
class SinkAudioOutputStream : public AudioOutputStream {
  static const int kDefaultSampleRateHz = 48000;
  static const size_t kDefaultFramesPerBuffer = 512;
  static const size_t kDefaultNumChannels = 2;

 public:
  explicit SinkAudioOutputStream(
      std::shared_ptr<util::Clock> clock = util::Clock::SystemClock(),
      int sample_rate_hz = kDefaultSampleRateHz,
      size_t frames_per_buffer = kDefaultFramesPerBuffer,
      size_t num_channels = kDefaultNumChannels);
  ~SinkAudioOutputStream() override;

  absl::Status StartPlayback(
      const AudioOutputStream::Callback& callback) override;

  absl::Status StopPlayback() override;

  int GetNumBufferUnderruns() const;

 private:
  void Run();

  std::shared_ptr<util::Clock> clock_;

  std::unique_ptr<std::thread> thread_ = nullptr;
  std::atomic<bool> running_ = false;
  std::vector<int16_t> buffer_;

  absl::Mutex callback_mutex_;
  AudioOutputStream::Callback ABSL_GUARDED_BY(callback_mutex_) callback_;

  std::atomic<int> buffer_underruns_ = 0;
};

}  // namespace guideline::audio

#endif  // PROJECT_GUIDELINE_AUDIO_SINK_AUDIO_OUTPUT_STREAM_H_
