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

#ifndef PROJECT_GUIDELINE_AUDIO_AUDIO_BUFFER_H_
#define PROJECT_GUIDELINE_AUDIO_AUDIO_BUFFER_H_

#include <cstdint>
#include <vector>

#include "absl/log/check.h"

namespace guideline::audio {

typedef std::vector<std::vector<float>> PlanarAudioBuffer;

// Convert the given float to an int16_t in the range
// [-32767 (-0x7FFF), 32767 (0x7FFF)]
inline void ConvertSampleFromFloatFormat(float input, int16_t* output) {
  DCHECK(output);
  constexpr float kInt16Min = static_cast<float>(-0x7FFF);
  constexpr float kInt16Max = static_cast<float>(0x7FFF);
  constexpr float kFloatToInt16 = kInt16Max;

  const float scaled_value = input * kFloatToInt16;
  const float clamped_value =
      std::min(kInt16Max, std::max(kInt16Min, scaled_value));
  *output = static_cast<int16_t>(clamped_value);
}

// Convert the given int16_t to a float in the range [-1.0f, 1.0f].
inline void ConvertSampleToFloatFormat(int16_t input, float* output) {
  DCHECK(output);
  constexpr float kInt16Max = static_cast<float>(0x7FFF);
  constexpr float kInt16ToFloat = 1.0f / kInt16Max;
  *output = input * kInt16ToFloat;
}

}  // namespace guideline::audio

#endif  // PROJECT_GUIDELINE_AUDIO_AUDIO_BUFFER_H_
