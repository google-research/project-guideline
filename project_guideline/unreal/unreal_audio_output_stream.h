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

#ifndef PROJECT_GUIDELINE_UNREAL_UNREAL_AUDIO_OUTPUT_STREAM_H_
#define PROJECT_GUIDELINE_UNREAL_UNREAL_AUDIO_OUTPUT_STREAM_H_

#include "absl/synchronization/mutex.h"
#include "project_guideline/audio/audio_output_stream.h"

namespace guideline::unreal {

class UnrealAudioOutputStream : public audio::AudioOutputStream {
 public:
  explicit UnrealAudioOutputStream(size_t frames_per_buffer);
  ~UnrealAudioOutputStream() override;

  absl::Status StartPlayback(
      const audio::AudioOutputStream::Callback& callback) override;
  absl::Status StopPlayback() override;

  int32_t OnGenerateAudio(int16_t* out_buffer, int32_t num_samples);

 private:
  absl::Mutex mutex_;
  audio::AudioOutputStream::Callback callback_ ABSL_GUARDED_BY(mutex_);
};

}  // namespace guideline::unreal

#endif  // PROJECT_GUIDELINE_UNREAL_UNREAL_AUDIO_OUTPUT_STREAM_H_
