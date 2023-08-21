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

#include "GuidelineMapShader.h"
#include "RHIUtilities.h"
#include "guideline_unreal_plugin.h"

class FGuidelineMapRenderer {
 public:
  FGuidelineMapRenderer(TSharedPtr<guideline::unreal::UnrealPlugin> Plugin)
      : GuidelinePlugin(Plugin){};

  void Start();
  void Stop();

  void Render(FRHICommandListImmediate& RHICmdList, FIntRect Viewport);

 private:
  void RenderOpaque(FPostOpaqueRenderParameters& Parameters);

  TSharedPtr<guideline::unreal::UnrealPlugin> GuidelinePlugin;
  FDelegateHandle RenderDelegateHandle;
  FDynamicReadBuffer LinePointBuffer;
  TArray<FVector4, SceneRenderingAllocator> LinePointData;
};
