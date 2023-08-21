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

#include "GuidelineMapRenderer.h"

#include "CommonRenderResources.h"
#include "EngineModule.h"
#include "Math/UnrealMathUtility.h"
#include "RenderGraphBuilder.h"
#include "RendererInterface.h"
#include "ScreenRendering.h"
#include "ShaderParameterStruct.h"

void FGuidelineMapRenderer::Start() {}

void FGuidelineMapRenderer::Stop() {
  LinePointBuffer.Release();
  LinePointData.Empty();
}

void FGuidelineMapRenderer::Render(FRHICommandListImmediate& RHICmdList,
                                   FIntRect Viewport) {
  SCOPED_DRAW_EVENT(RHICmdList, GuidelineMapRender);

  int MapSize = (int)(Viewport.Height() * 0.4);
  int MapMargin = (int)(Viewport.Height() * 0.05);
  int MapX = Viewport.Min.X + Viewport.Width() - MapMargin - MapSize;
  int MapY = Viewport.Min.Y + Viewport.Height() - MapMargin - MapSize;

  const auto ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);

  FGuidelineMapPS::FParameters MapParams;
  MapParams.Viewport = FVector4(MapX, MapY, MapSize, MapSize);

  auto RunnerPose = GuidelinePlugin->GetRunnerPose();
  FVector2D RunnerPosition =
      FVector2D(RunnerPose.position.x, RunnerPose.position.y);
  FVector2D RunnerVector =
      FVector2D(RunnerPose.direction.x, RunnerPose.direction.y);

  auto ControlSignal = GuidelinePlugin->GetLastControlSignal();

  MapParams.RunnerPosition = RunnerPosition;
  MapParams.RunnerVector = RunnerVector;
  MapParams.RotationMovementRadians =
      ControlSignal.rotation_movement_degrees * (PI / 180.0f);
  MapParams.LateralMovementMeters = ControlSignal.lateral_movement_meters;
  MapParams.TurnPoint =
      FVector2D(ControlSignal.turn_point.x, ControlSignal.turn_point.y);
  MapParams.TurnAngleRadians = ControlSignal.turn_angle_degrees * (PI / 180.0f);
  MapParams.Stop = ControlSignal.stop ? 1 : 0;

  guideline::unreal::GuidelinePoints GuidelinePoints;
  GuidelinePlugin->GetGuidelinePoints(&GuidelinePoints);
  LinePointData.Empty();
  const uint kMaxPoints = 100;
  for (int i = 0; i < GuidelinePoints.num_points && i < kMaxPoints; ++i) {
    auto& point = GuidelinePoints.points[i];
    LinePointData.Add(FVector4(point.x, point.y, 0.f, 0.f));
  }
  GuidelinePlugin->ReleaseGuidelinePoints(&GuidelinePoints);

  uint RequiredBytes = sizeof(FVector4) * kMaxPoints;
  if (LinePointBuffer.NumBytes < RequiredBytes) {
    LinePointBuffer.Initialize(sizeof(FVector4), kMaxPoints, PF_A32B32G32R32F,
                               BUF_Volatile);
  }
  LinePointBuffer.Lock();
  FPlatformMemory::Memcpy(LinePointBuffer.MappedBuffer, LinePointData.GetData(),
                          LinePointData.Num() * LinePointData.GetTypeSize());
  LinePointBuffer.Unlock();

  MapParams.LinePoints = LinePointBuffer.SRV;
  MapParams.LinePointCount = LinePointData.Num();

  float YawDegrees =
      FMath::Atan2(RunnerVector.Y, RunnerVector.X) * (180.f / PI) + 90;

  MapParams.TransformMatrix = FRotationAboutPointMatrix::Make(
      FRotator(0.0, YawDegrees + 180, 0.0), FVector(0.5, 0.5, 0.0));

  TShaderMapRef<FScreenVS> VertexShader(ShaderMap);
  TShaderMapRef<FGuidelineMapPS> PixelShader(ShaderMap);

  RHICmdList.SetViewport(MapX, MapY, 0.0f, MapX + MapSize, MapY + MapSize,
                         1.0f);

  FGraphicsPipelineStateInitializer GraphicsPSOInit;
  RHICmdList.ApplyCachedRenderTargets(GraphicsPSOInit);
  GraphicsPSOInit.BlendState =
      TStaticBlendState<CW_RGB, BO_Add, BF_SourceAlpha,
                        BF_InverseSourceAlpha>::GetRHI();
  GraphicsPSOInit.RasterizerState =
      TStaticRasterizerState<FM_Solid, CM_None>::GetRHI();
  GraphicsPSOInit.DepthStencilState =
      TStaticDepthStencilState<false, CF_Always>::GetRHI();

  extern TGlobalResource<FFilterVertexDeclaration> GFilterVertexDeclaration;
  GraphicsPSOInit.BoundShaderState.VertexDeclarationRHI =
      GFilterVertexDeclaration.VertexDeclarationRHI;
  GraphicsPSOInit.BoundShaderState.VertexShaderRHI =
      VertexShader.GetVertexShader();
  GraphicsPSOInit.BoundShaderState.PixelShaderRHI =
      PixelShader.GetPixelShader();
  GraphicsPSOInit.PrimitiveType = PT_TriangleList;

  SetGraphicsPipelineState(RHICmdList, GraphicsPSOInit);

  SetShaderParameters(RHICmdList, PixelShader, PixelShader.GetPixelShader(),
                      MapParams);

  GetRendererModule().DrawRectangle(
      RHICmdList, 0, 0, MapSize, MapSize, 0, 0, MapSize, MapSize,
      FIntPoint(MapSize, MapSize), FIntPoint(MapSize, MapSize), VertexShader);
}
