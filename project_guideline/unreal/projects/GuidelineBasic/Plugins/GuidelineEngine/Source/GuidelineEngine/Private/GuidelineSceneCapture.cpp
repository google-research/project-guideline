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

// Fill out your copyright notice in the Description page of Project Settings.

#include "GuidelineSceneCapture.h"

#include <Runtime/Engine/Public/ShowFlags.h>

#include "Kismet/GameplayStatics.h"
#include "Runtime/Engine/Classes/Components/SceneCaptureComponent2D.h"
#include "Runtime/Engine/Classes/Engine/TextureRenderTarget2D.h"

AGuidelineSceneCapture::AGuidelineSceneCapture(
    const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer) {
  PrimaryActorTick.bCanEverTick = true;
}

void AGuidelineSceneCapture::PostInitializeComponents() {
  Super::PostInitializeComponents();

  UE_LOG(LogTemp, Warning, TEXT("SceneCapturePostInitializeComponents"));
}

void AGuidelineSceneCapture::SetGuidelinePlugin(
    guideline::unreal::UnrealPlugin* Plugin) {
  GuidelinePlugin = Plugin;

  float ImageWidth;
  float ImageHeight;
  GuidelinePlugin->GetCameraImageDimensions(ImageWidth, ImageHeight);

  UE_LOG(LogTemp, Warning,
         TEXT("Initializing render target width: %d height: %d"), ImageWidth,
         ImageHeight);

  UTextureRenderTarget2D* RenderTarget2D = NewObject<UTextureRenderTarget2D>();
  RenderTarget2D->InitCustomFormat(ImageWidth, ImageHeight, PF_B8G8R8A8, true);
  RenderTarget2D->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
  RenderTarget2D->bGPUSharedFlag = true;

  GetCaptureComponent2D()->TextureTarget = RenderTarget2D;

  GetCaptureComponent2D()->CaptureSource =
      ESceneCaptureSource::SCS_FinalColorLDR;
  GetCaptureComponent2D()->ShowFlags.SetTemporalAA(true);

  FMatrix ProjectionMatrix;
  GuidelinePlugin->GetProjectionMatrix(GNearClippingPlane, 100.0f,
                                       ProjectionMatrix.M[0]);

  UE_LOG(LogTemp, Warning,
         TEXT("Setting dynamic projection matrix [%0.4f, %0.4f, %0.4f, %0.4f] "
              "[%0.4f, %0.4f, %0.4f, %0.4f] [%0.4f, %0.4f, %0.4f, %0.4f] "
              "[%0.4f, %0.4f, %0.4f, %0.4f]"),
         ProjectionMatrix.M[0][0], ProjectionMatrix.M[0][1],
         ProjectionMatrix.M[0][2], ProjectionMatrix.M[0][3],

         ProjectionMatrix.M[1][0], ProjectionMatrix.M[1][1],
         ProjectionMatrix.M[1][2], ProjectionMatrix.M[1][3],

         ProjectionMatrix.M[2][0], ProjectionMatrix.M[2][1],
         ProjectionMatrix.M[2][2], ProjectionMatrix.M[2][3],

         ProjectionMatrix.M[3][0], ProjectionMatrix.M[3][1],
         ProjectionMatrix.M[3][2], ProjectionMatrix.M[3][3]);

  // We need to multiply the center offset by -1 to get the correct projection
  // matrix in Unreal.
  ProjectionMatrix.M[2][0] = -1.0f * ProjectionMatrix.M[2][0];
  ProjectionMatrix.M[2][1] = -1.0f * ProjectionMatrix.M[2][1];

  // Unreal uses the infinite far plane project matrix.
  ProjectionMatrix.M[2][2] = 0.0f;
  ProjectionMatrix.M[2][3] = 1.0f;
  ProjectionMatrix.M[3][2] = GNearClippingPlane;

  GetCaptureComponent2D()->bUseCustomProjectionMatrix = true;
  GetCaptureComponent2D()->CustomProjectionMatrix = ProjectionMatrix;
}

void AGuidelineSceneCapture::Tick(float DeltaTime) {
  Super::Tick(DeltaTime);

  APlayerCameraManager* CameraManager =
      UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0);
  GetCaptureComponent2D()->SetWorldLocationAndRotationNoPhysics(
      CameraManager->GetCameraLocation(), CameraManager->GetCameraRotation());

  bool HasPreviousCameraPose = !CameraTransformQueue.IsEmpty();

  FTransform CameraTransform(CameraManager->GetCameraRotation(),
                             CameraManager->GetCameraLocation());
  CameraTransformQueue.Enqueue(CameraTransform);

  // Render requests are delayed by 1 frame so we need the previous camera pose
  // to match the rendered image.
  if (!HasPreviousCameraPose) {
    return;
  }

  CameraTransformQueue.Dequeue(CameraTransform);
  FTextureRenderTargetResource* RenderTargetResource =
      GetCaptureComponent2D()
          ->TextureTarget->GameThread_GetRenderTargetResource();

  struct FReadSurfaceContext {
    FRenderTarget* SrcRenderTarget;
    TArray<FColor>* OutData;
    FIntRect Rect;
    FReadSurfaceDataFlags Flags;
  };

  TSharedPtr<FRenderRequestStruct> RenderRequest =
      MakeShared<FRenderRequestStruct>();
  float RealtimeSeconds = UGameplayStatics::GetRealTimeSeconds(GetWorld());
  RenderRequest->TimestampUs = (int64)(RealtimeSeconds * 1000000);
  RenderRequest->CameraTransform = CameraTransform;
  RenderRequest->ImageWidth = RenderTargetResource->GetSizeXY().X;
  RenderRequest->ImageHeight = RenderTargetResource->GetSizeXY().Y;

  FReadSurfaceContext readSurfaceContext = {
      RenderTargetResource, &(RenderRequest->Image),
      FIntRect(0, 0, RenderRequest->ImageWidth, RenderRequest->ImageHeight),
      FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

  ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)
  ([readSurfaceContext](FRHICommandListImmediate& RHICmdList) {
    RHICmdList.ReadSurfaceData(
        readSurfaceContext.SrcRenderTarget->GetRenderTargetTexture(),
        readSurfaceContext.Rect, *readSurfaceContext.OutData,
        readSurfaceContext.Flags);
  });

  RenderRequestQueue.Enqueue(RenderRequest);
  RenderRequest->RenderFence.BeginFence();

  if (!RenderRequestQueue.IsEmpty()) {
    TSharedPtr<FRenderRequestStruct> NextRenderRequest;
    RenderRequestQueue.Peek(NextRenderRequest);

    if (NextRenderRequest->RenderFence.IsFenceComplete()) {
      if (GuidelinePlugin) {
        FMatrix UnrealPoseMatrix =
            NextRenderRequest->CameraTransform.ToMatrixNoScale();
        UnrealPoseMatrix.SetOrigin(UnrealPoseMatrix.GetOrigin() / 100.0);

        FVector Origin = UnrealPoseMatrix.GetOrigin();
        FQuat Quat = UnrealPoseMatrix.ToQuat();

        GuidelinePlugin->OnTracking(true);
        GuidelinePlugin->OnCameraPose(NextRenderRequest->TimestampUs, Origin.X,
                                      Origin.Y, Origin.Z, Quat.X, Quat.Y,
                                      Quat.Z, Quat.W);
        GuidelinePlugin->OnImage(
            NextRenderRequest->TimestampUs, NextRenderRequest->ImageWidth,
            NextRenderRequest->ImageHeight, NextRenderRequest->Image.GetData());
      }
      RenderRequestQueue.Pop();
    }
  }
}
