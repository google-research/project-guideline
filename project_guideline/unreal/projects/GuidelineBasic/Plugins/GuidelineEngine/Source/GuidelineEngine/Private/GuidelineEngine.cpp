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

#include "GuidelineEngine.h"

#include "Core.h"
#include "GuidelineSoundWaveProcedural.h"
#include "Interfaces/IPluginManager.h"
#include "Modules/ModuleManager.h"

#define LOCTEXT_NAMESPACE "FGuidelineEngineModule"

void FGuidelineEngineModule::StartupModule() {
  UE_LOG(LogTemp, Warning, TEXT("Guideline StartupModule"));
  // Get the base directory of this plugin
  FString BaseDir =
      IPluginManager::Get().FindPlugin("GuidelineEngine")->GetBaseDir();

  // Add on the relative location of the third party dll and load it
  FString LibraryPath = FPaths::Combine(
      *BaseDir, TEXT("Source/GuidelineEngine/lib/guideline_engine_unreal.so"));

  LibraryHandle = !LibraryPath.IsEmpty()
                      ? FPlatformProcess::GetDllHandle(*LibraryPath)
                      : nullptr;

  if (!LibraryHandle) {
    FMessageDialog::Open(
        EAppMsgType::Ok,
        LOCTEXT("GuidelinePluginError", "Failed to load Guideline Engine"));
  }
}

void FGuidelineEngineModule::ShutdownModule() {
  UE_LOG(LogTemp, Warning, TEXT("Guideline ShutdownModule"));
  if (LibraryHandle) {
    FPlatformProcess::FreeDllHandle(LibraryHandle);
    LibraryHandle = nullptr;
  }
}

bool FGuidelineEngineModule::IsGuidelineEngineLoaded() {
  return LibraryHandle != nullptr;
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FGuidelineEngineModule, GuidelineEngine)
