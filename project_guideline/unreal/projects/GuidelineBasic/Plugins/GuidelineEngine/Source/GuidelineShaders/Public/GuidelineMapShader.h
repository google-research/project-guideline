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

#include "GlobalShader.h"
#include "Math/Vector4.h"
#include "Shader.h"
#include "ShaderParameterStruct.h"
#include "ShaderParameters.h"
#include "VectorTypes.h"

class GUIDELINESHADERS_API FGuidelineMapPS : public FGlobalShader {
  DECLARE_GLOBAL_SHADER(FGuidelineMapPS);
  SHADER_USE_PARAMETER_STRUCT(FGuidelineMapPS, FGlobalShader);

  static bool ShouldCompilePermutation(
      const FGlobalShaderPermutationParameters& Parameters) {
    return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
  }

  static bool ShouldCache(EShaderPlatform Platform) { return false; }

  BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
  SHADER_PARAMETER(FVector4, Viewport)
  SHADER_PARAMETER(FVector2D, RunnerPosition)
  SHADER_PARAMETER(FVector2D, RunnerVector)
  SHADER_PARAMETER(float, RotationMovementRadians)
  SHADER_PARAMETER(float, LateralMovementMeters)
  SHADER_PARAMETER(FVector2D, TurnPoint)
  SHADER_PARAMETER(float, TurnAngleRadians)
  SHADER_PARAMETER(uint, Stop)
  SHADER_PARAMETER_SRV(StrongTypedBuffer<float4>, LinePoints)
  SHADER_PARAMETER(uint, LinePointCount)
  SHADER_PARAMETER(FMatrix, TransformMatrix)
  END_SHADER_PARAMETER_STRUCT()

  void SetParameters(FRHICommandList& RHICmdList,
                     const FGuidelineMapPS::FParameters& Params);
};
