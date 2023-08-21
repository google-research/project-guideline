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

#ifndef PROJECT_GUIDELINE_ANDROID_AUDIO_AAUDIO_OUTPUT_STREAM_H_
#define PROJECT_GUIDELINE_ANDROID_AUDIO_AAUDIO_OUTPUT_STREAM_H_

#include <aaudio/AAudio.h>

#include <memory>
#include <thread>  // NOLINT

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "project_guideline/audio/audio_output_stream.h"

namespace guideline::audio {

// Implementation of AudioOutputStream that uses Android AAudio API.
class AndroidAudioOutputStream : public AudioOutputStream {
 public:
  static absl::StatusOr<std::unique_ptr<AndroidAudioOutputStream>> Create(
      size_t frames_per_buffer, int sample_rate_hz);
  ~AndroidAudioOutputStream() override;

  absl::Status StartPlayback(
      const AudioOutputStream::Callback& callback) override;

  absl::Status StopPlayback() override;

 private:
  static aaudio_data_callback_result_t DataCallback(AAudioStream* stream,
                                                    void* user_data,
                                                    void* audio_data,
                                                    int32_t num_frames);

  static void ErrorCallback(AAudioStream* stream, void* user_data,
                            aaudio_result_t error);

  AndroidAudioOutputStream(size_t num_channels, size_t frames_per_buffer,
                           int sample_rate_hz,
                           AAudioStreamBuilder* stream_builder);

  aaudio_data_callback_result_t OnData(AAudioStream* stream, void* audio_data,
                                       int32_t num_frames);

  void OnError(AAudioStream* stream, aaudio_result_t error);

  absl::Status StartStream() ABSL_EXCLUSIVE_LOCKS_REQUIRED(run_mutex_);
  absl::Status StopStream() ABSL_EXCLUSIVE_LOCKS_REQUIRED(run_mutex_);

  void RestartStream();

  AAudioStreamBuilder* stream_builder_;

  AAudioStream* aaudio_stream_ = nullptr;

  absl::Mutex run_mutex_;
  absl::Mutex callback_mutex_;
  AudioOutputStream::Callback ABSL_GUARDED_BY(callback_mutex_) callback_;

  absl::Mutex restart_thread_mutex_;
  std::unique_ptr<std::thread> restart_thread_
      ABSL_GUARDED_BY(restart_thread_mutex_) = nullptr;
};

}  // namespace guideline::audio

#endif  // PROJECT_GUIDELINE_ANDROID_AUDIO_AAUDIO_OUTPUT_STREAM_H_
