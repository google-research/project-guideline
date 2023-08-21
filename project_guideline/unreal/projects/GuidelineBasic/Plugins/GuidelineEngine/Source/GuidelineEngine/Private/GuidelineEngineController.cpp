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

#include "GuidelineEngineController.h"

#include "AudioDevice.h"
#include "Components/AudioComponent.h"
#include "Engine/GameEngine.h"
#include "GameFramework/HUD.h"
#include "GuidelineEngine.h"
#include "GuidelineMapShader.h"
#include "GuidelineSoundWaveProcedural.h"
#include "Kismet/GameplayStatics.h"
#include "Modules/ModuleManager.h"
#include "RHICommandList.h"

AGuidelineEngineController::AGuidelineEngineController() {
  PrimaryActorTick.bCanEverTick = true;
}

void AGuidelineEngineController::BeginPlay() {
  Super::BeginPlay();

  static FName GuidelineModuleName("GuidelineEngine");
  FGuidelineEngineModule& GuidelineModule =
      FModuleManager::GetModuleChecked<FGuidelineEngineModule>(
          GuidelineModuleName);
  if (!GuidelineModule.IsGuidelineEngineLoaded()) {
    UE_LOG(LogTemp, Error, TEXT("Guideline Engine is not loaded"));
    return;
  }

  FAudioDeviceHandle AudioDevice = GEngine->GetMainAudioDevice();
  if (!AudioDevice) {
    UE_LOG(LogTemp, Error, TEXT("Guideline Engine no audio device"));
    return;
  }

  UGuidelineSoundWaveProcedural* SoundWave =
      NewObject<UGuidelineSoundWaveProcedural>(this);

  guideline::unreal::UnrealGuidelineOptions options;
  options.audio_frames_per_buffer = AudioDevice->GetBufferLength();

  std::string file_logger_output_dir = std::string(TCHAR_TO_UTF8(
      *FPaths::ConvertRelativePathToFull(FPaths::GeneratedConfigDir())));
  options.file_logger_output_dir = file_logger_output_dir.c_str();

  options.enable_conservative_mode = true;
  options.conservative_mode_lane_width_meters = 1.2;

  std::string config_dir = std::string(TCHAR_TO_UTF8(
      *FPaths::ConvertRelativePathToFull(FPaths::GeneratedConfigDir())));
  options.config_dir = config_dir.c_str();

  GuidelinePlugin = TSharedPtr<guideline::unreal::UnrealPlugin>(
      guideline::unreal::CreateUnrealPlugin(options));

  GuidelinePlugin->SetDebugLogListener(&Logger);
  GuidelinePlugin->SetControlSignalListener(&ControlSignalListener);

  SoundWave->SetGuidelinePlugin(GuidelinePlugin.Get());
  SoundWave->SetSampleRate(48000);
  SoundWave->NumChannels = 2;
  SoundWave->Duration = INDEFINITELY_LOOPING_DURATION;
  SoundWave->SoundGroup = SOUNDGROUP_Default;
  SoundWave->bLooping = false;

  UGameplayStatics::PlaySound2D(GetWorld(), SoundWave);
  UE_LOG(LogTemp, Warning, TEXT("Playing Audio"));

  SceneCapture = GetWorld()->SpawnActor<AGuidelineSceneCapture>();
  SceneCapture->SetGuidelinePlugin(GuidelinePlugin.Get());
  UE_LOG(LogTemp, Warning, TEXT("Created Scene Capture"));

  GuidelinePlugin->Start();

  MapRenderer = MakeUnique<FGuidelineMapRenderer>(GuidelinePlugin);
  MapRenderer->Start();

  APlayerController* PlayerController =
      UGameplayStatics::GetPlayerController(this, 0);
  AHUD* HUD = PlayerController->GetHUD();
  HUD->AddPostRenderedActor(this);
  HUD->bShowOverlays = true;

  Playing = true;
}

void AGuidelineEngineController::EndPlay(
    const EEndPlayReason::Type EndPlayReason) {
  Super::EndPlay(EndPlayReason);

  Playing = false;

  if (MapRenderer) {
    MapRenderer->Stop();
    MapRenderer = nullptr;
  }

  if (SceneCapture) {
    SceneCapture = nullptr;
  }

  if (GuidelinePlugin.IsValid()) {
    GuidelinePlugin->Stop();
    GuidelinePlugin = nullptr;
  }
}

void AGuidelineEngineController::Tick(float DeltaTime) {
  Super::Tick(DeltaTime);
}

void AGuidelineEngineController::PostRenderFor(APlayerController* PC,
                                               UCanvas* Canvas,
                                               FVector CameraPosition,
                                               FVector CameraDir) {
  Super::PostRenderFor(PC, Canvas, CameraPosition, CameraDir);

  auto ViewportSize = GEngine->GameViewport->Viewport->GetSizeXY();

  int StartX = 0;
  int StartY = 0;
  if (Canvas->SizeX < ViewportSize.X) {
    StartX = (ViewportSize.X - Canvas->SizeX) / 2;
  }
  if (Canvas->SizeY < ViewportSize.Y) {
    StartY = (ViewportSize.Y - Canvas->SizeY) / 2;
  }
  FIntRect HUDViewport;
  HUDViewport.Min = FIntPoint(StartX, StartY);
  HUDViewport.Max = FIntPoint(StartX + Canvas->SizeX, StartY + Canvas->SizeY);

  AGuidelineEngineController* Self = this;
  ENQUEUE_RENDER_COMMAND(RenderMap)
  ([Self, HUDViewport](FRHICommandListImmediate& RHICmdList) {
    Self->RenderHUD(RHICmdList, HUDViewport);
  });
}

void AGuidelineEngineController::RenderHUD(FRHICommandListImmediate& RHICmdList,
                                           FIntRect Viewport) {
  if (!Playing) {
    return;
  }
  MapRenderer->Render(RHICmdList, Viewport);
}

void DebugLogger::OnDebugLog(const char* message) {
  FString Msg(message);
  UE_LOG(LogTemp, Display, TEXT("GLE: %s"), *Msg);
}

void ControlListener::OnControlSignal(
    const guideline::unreal::UnrealControlSignal& control_signal) {
  /*UE_LOG(LogTemp, Display, TEXT("Control: stop: %d lat: %f rot: %f
     turn_point_distance: %f turn_angle: %f"), control_signal.stop ? 0 : 1,
         control_signal.lateral_movement_meters,
     control_signal.rotation_movement_degrees,
         control_signal.turn_point_distance_meters,
         control_signal.turn_angle_degrees);*/
}
