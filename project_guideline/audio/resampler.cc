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

#include "project_guideline/audio/resampler.h"

#include <cstddef>
#include <vector>

#include "absl/log/check.h"

namespace guideline::audio {

namespace {
// 4-point, 3rd order Hermite interpolation.
// See 'Polynomial Interpolators for High-Quality Resampling of Oversampled
// Audio' by Olli Niemitalo.
inline float Interpolate(float s0, float s1, float s2, float s3, float t) {
  float c0 = s1;
  float c1 = .5 * (s2 - s0);
  float c2 = s0 - (2.5 * s1) + (2 * s2) - (.5 * s3);
  float c3 = (.5 * (s3 - s0)) + (1.5 * (s1 - s2));
  return (((((c3 * t) + c2) * t) + c1) * t) + c0;
}
}  // namespace

Resampler::Resampler(int input_sample_rate_hz, int output_sample_rate_hz)
    : input_sample_rate_hz_(input_sample_rate_hz),
      output_sample_rate_hz_(output_sample_rate_hz),
      playback_rate_(1.0f * input_sample_rate_hz / output_sample_rate_hz) {}

void Resampler::Resample(const std::vector<float>& input,
                         std::vector<float>& output) {
  size_t out_size = static_cast<size_t>(input.size() / playback_rate_);
  output.resize(out_size);
  size_t unused;
  size_t num_frames_copied =
      ResampleChunk(input, output, playback_rate_, 0, 0, out_size, unused);
  CHECK_EQ(num_frames_copied, out_size);
}

size_t Resampler::ResampleChunk(const std::vector<float>& input,
                                std::vector<float>& output, float playback_rate,
                                size_t start_input_index,
                                size_t start_output_index,
                                size_t max_frames_to_copy,
                                size_t& num_frames_advanced) {
  size_t num_frames_copied = 0;
  const size_t num_input_frames = input.size();
  size_t output_index = start_output_index;
  const float start_input_index_float =
      std::ceil(start_input_index / playback_rate) * playback_rate;
  num_frames_advanced = 0;
  for (int i = 0; i < max_frames_to_copy; ++i) {
    float input_index_float = start_input_index_float + i * playback_rate;
    size_t new_input_index = static_cast<size_t>(input_index_float);
    if (new_input_index >= num_input_frames) {
      num_frames_advanced = (num_input_frames - start_input_index);
      break;
    }
    num_frames_advanced = (new_input_index - start_input_index) + 1;
    float t = input_index_float - new_input_index;
    const float* input_ptr = &input[new_input_index];
    float s1 = *input_ptr;
    float s0 = new_input_index > 0 ? *(input_ptr - 1) : s1;
    float s2 = new_input_index < num_input_frames - 2 ? *(input_ptr + 1) : s1;
    float s3 = new_input_index < num_input_frames - 3 ? *(input_ptr + 2) : s2;
    output[output_index++] = Interpolate(s0, s1, s2, s3, t);
    ++num_frames_copied;
  }
  return num_frames_copied;
}

}  // namespace guideline::audio
