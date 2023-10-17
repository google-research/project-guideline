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

#ifndef PROJECT_GUIDELINE_AUDIO_RESAMPLER_H_
#define PROJECT_GUIDELINE_AUDIO_RESAMPLER_H_

#include <cstddef>
#include <vector>

namespace guideline::audio {

// Simple audio resampler.
// This resampler is capable of converting an audio stream to a desired sample
// rate. It can also be used to dynamically vary the pitch of an audio stream
// through calls to ResampleChunk.
class Resampler {
 public:
  Resampler(int input_sample_rate_hz, int output_sample_rate_hz);
  void Resample(const std::vector<float>& input, std::vector<float>& output);

  // Resamples a chunk of input to output at the given playback rate. This
  // may be used to dynamically pitch shift the input, if the output is
  // interpreted at the same sample rate as input.
  // Returns the number of frames copied to the output buffer.
  // num_frames_advanced will be set to the number of input frames used, which
  // may be different than the number of output frames when playback_rate != 1.0
  static size_t ResampleChunk(const std::vector<float>& input,
                              std::vector<float>& output, float playback_rate,
                              size_t start_input_index,
                              size_t start_output_index,
                              size_t max_frames_to_copy,
                              size_t& num_frames_advanced);

  int input_sample_rate_hz() const { return input_sample_rate_hz_; }
  int output_sample_rate_hz() const { return output_sample_rate_hz_; }

 private:
  const int input_sample_rate_hz_;
  const int output_sample_rate_hz_;
  const float playback_rate_;
};

}  // namespace guideline::audio

#endif  // PROJECT_GUIDELINE_AUDIO_RESAMPLER_H_
