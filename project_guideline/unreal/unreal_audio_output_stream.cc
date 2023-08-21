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

#include "project_guideline/unreal/unreal_audio_output_stream.h"

#include "absl/status/status.h"

namespace guideline::unreal {

namespace {
constexpr int kSampleRate = 48000;
constexpr size_t kNumChannels = 2;
}  // namespace

UnrealAudioOutputStream::UnrealAudioOutputStream(size_t frames_per_buffer)
    : audio::AudioOutputStream(kSampleRate, frames_per_buffer, kNumChannels) {}

UnrealAudioOutputStream::~UnrealAudioOutputStream() {
  StopPlayback().IgnoreError();
}

absl::Status UnrealAudioOutputStream::StartPlayback(
    const audio::AudioOutputStream::Callback& callback) {
  absl::MutexLock lock(&mutex_);
  callback_ = callback;
  return absl::OkStatus();
}

absl::Status UnrealAudioOutputStream::StopPlayback() {
  absl::MutexLock lock(&mutex_);
  callback_ = nullptr;
  return absl::OkStatus();
}

int32_t UnrealAudioOutputStream::OnGenerateAudio(int16_t* out_buffer,
                                                 int32_t num_samples) {
  audio::AudioOutputStream::Callback callback;
  {
    absl::MutexLock lock(&mutex_);
    callback = callback_;
  }
  if (!callback || !callback(out_buffer, kNumChannels, num_samples)) {
    memset(out_buffer, 0, sizeof(int16_t) * num_samples * kNumChannels);
  }
  return num_samples;
}

}  // namespace guideline::unreal
