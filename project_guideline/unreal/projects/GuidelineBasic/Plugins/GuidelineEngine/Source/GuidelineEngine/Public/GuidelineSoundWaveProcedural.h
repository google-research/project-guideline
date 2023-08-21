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

#pragma once

#include "CoreMinimal.h"
#include "Sound/SoundWaveProcedural.h"
#include "guideline_unreal_plugin.h"

// clang-format off
#include "GuidelineSoundWaveProcedural.generated.h"  // Must be last
// clang-format on

UCLASS()
class GUIDELINEENGINE_API UGuidelineSoundWaveProcedural
    : public USoundWaveProcedural {
  GENERATED_BODY()

 public:
  void SetGuidelinePlugin(guideline::unreal::UnrealPlugin* Plugin);

  int32 GeneratePCMData(uint8* PCMData, const int32 SamplesNeeded) override;

  int32 GetNumSamplesToGeneratePerCallback();

 private:
  guideline::unreal::UnrealPlugin* GuidelinePlugin = nullptr;
};
