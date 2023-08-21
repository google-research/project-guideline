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

#include "project_guideline/audio/sink_audio_output_stream.h"

#include <memory>

#include "absl/functional/bind_front.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "project_guideline/audio/audio_output_stream.h"
#include "project_guideline/util/clock.h"

namespace guideline::audio {

SinkAudioOutputStream::SinkAudioOutputStream(std::shared_ptr<util::Clock> clock,
                                             int sample_rate_hz,
                                             size_t frames_per_buffer,
                                             size_t num_channels)
    : AudioOutputStream(sample_rate_hz, frames_per_buffer, num_channels),
      clock_(clock) {}

SinkAudioOutputStream::~SinkAudioOutputStream() {
  auto status = StopPlayback();
  if (!status.ok()) {
    LOG(ERROR) << "Failed to stop playback: " << status;
  }
}

absl::Status SinkAudioOutputStream::StartPlayback(
    const AudioOutputStream::Callback& callback) {
  if (!running_.exchange(true)) {
    CHECK(thread_ == nullptr);

    {
      absl::MutexLock lock(&callback_mutex_);
      callback_ = callback;
    }

    buffer_underruns_ = 0;
    thread_ = std::make_unique<std::thread>(
        absl::bind_front(&SinkAudioOutputStream::Run, this));
  }
  return absl::OkStatus();
}

absl::Status SinkAudioOutputStream::StopPlayback() {
  if (running_.exchange(false)) {
    thread_->join();
    thread_.reset();
    {
      absl::MutexLock lock(&callback_mutex_);
      callback_ = nullptr;
    }
  }
  return absl::OkStatus();
}

int SinkAudioOutputStream::GetNumBufferUnderruns() const {
  return buffer_underruns_.load();
}

void SinkAudioOutputStream::Run() {
  buffer_.resize(GetNumChannels() * GetFramesPerBuffer());

  double buffer_rate_hz = (double)GetSampleRateHz() / GetFramesPerBuffer();
  auto buffer_duration = absl::Seconds(1.0 / buffer_rate_hz);
  auto next_buffer_time = clock_->Now();
  while (running_.load()) {
    audio::AudioOutputStream::Callback callback;
    {
      absl::MutexLock lock(&callback_mutex_);
      callback = callback_;
    }
    if (!callback) {
      return;
    }
    callback(buffer_.data(), GetNumChannels(), GetFramesPerBuffer());
    next_buffer_time += buffer_duration;
    if (next_buffer_time - clock_->Now() < absl::ZeroDuration()) {
      ++buffer_underruns_;
    } else {
      clock_->SleepUntil(next_buffer_time);
    }
  }
}

}  // namespace guideline::audio
