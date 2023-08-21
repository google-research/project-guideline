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
#include "Engine/Canvas.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerController.h"
#include "GuidelineMapRenderer.h"
#include "GuidelineSceneCapture.h"
#include "HAL/ThreadSafeBool.h"
#include "guideline_unreal_plugin.h"

// clang-format off
#include "GuidelineEngineController.generated.h"  // Must be last
// clang-format on

class DebugLogger : public guideline::unreal::UnrealDebugLogListener {
 public:
  void OnDebugLog(const char* message);
};

class ControlListener : public guideline::unreal::UnrealControlSignalListener {
 public:
  void OnControlSignal(
      const guideline::unreal::UnrealControlSignal& control_signal) override;
};

UCLASS()
class GUIDELINEENGINE_API AGuidelineEngineController : public AActor {
  GENERATED_BODY()

 public:
  // Sets default values for this actor's properties
  AGuidelineEngineController();

 protected:
  // Called when the game starts or when spawned
  virtual void BeginPlay() override;
  virtual void EndPlay(const EEndPlayReason::Type EndPlayReas) override;

  virtual void PostRenderFor(APlayerController* PC, UCanvas* Canvas,
                             FVector CameraPosition,
                             FVector CameraDir) override;

 public:
  // Called every frame
  virtual void Tick(float DeltaTime) override;

 private:
  void RenderHUD(FRHICommandListImmediate& RHICmdList, FIntRect Viewport);

  TSharedPtr<guideline::unreal::UnrealPlugin> GuidelinePlugin;
  UAudioComponent* AudioComponent = nullptr;
  AGuidelineSceneCapture* SceneCapture = nullptr;
  DebugLogger Logger;
  ControlListener ControlSignalListener;
  TUniquePtr<FGuidelineMapRenderer> MapRenderer;
  FThreadSafeBool Playing = false;
};
