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
#include "Engine/SceneCapture2D.h"
#include "guideline_unreal_plugin.h"

// clang-format off
#include "GuidelineSceneCapture.generated.h"  // Must be last
// clang-format on

USTRUCT()
struct FRenderRequestStruct {
  GENERATED_BODY()

  uint64 TimestampUs;
  FTransform CameraTransform;
  TArray<FColor> Image;
  FRenderCommandFence RenderFence;
  uint32 ImageWidth;
  uint32 ImageHeight;

  FRenderRequestStruct() {}
};

UCLASS()
class GUIDELINEENGINE_API AGuidelineSceneCapture : public ASceneCapture2D {
  GENERATED_BODY()

  AGuidelineSceneCapture(const FObjectInitializer& ObjectInitializer);

 public:
  void SetGuidelinePlugin(guideline::unreal::UnrealPlugin* Plugin);

  virtual void PostInitializeComponents() override;
  virtual void Tick(float DeltaTime) override;

 private:
  guideline::unreal::UnrealPlugin* GuidelinePlugin = nullptr;
  TQueue<TSharedPtr<FRenderRequestStruct>> RenderRequestQueue;
  int32 QueueSize = 0;
  TQueue<FTransform> CameraTransformQueue;
};
