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

#include "GuidelineSoundWaveProcedural.h"

void UGuidelineSoundWaveProcedural::SetGuidelinePlugin(
    guideline::unreal::UnrealPlugin* Plugin) {
  GuidelinePlugin = Plugin;
}

int32 UGuidelineSoundWaveProcedural::GeneratePCMData(
    uint8* PCMData, const int32 SamplesNeeded) {
  if (GuidelinePlugin) {
    int32 generated = GuidelinePlugin->OnGenerateAudio(
        (int16*)PCMData, NumSamplesToGeneratePerCallback);
    return generated * NumChannels * sizeof(int16);
  }
  return 0;
}

int32 UGuidelineSoundWaveProcedural::GetNumSamplesToGeneratePerCallback() {
  return NumSamplesToGeneratePerCallback;
}
