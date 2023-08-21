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

using UnrealBuildTool;
using System.IO;

public class GuidelineEngine : ModuleRules {
  public GuidelineEngine(ReadOnlyTargetRules Target) : base(Target) {
    PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
    CppStandard = CppStandardVersion.Cpp17;

    PublicAdditionalLibraries.Add(
        Path.Combine(ModuleDirectory, "lib", "libguideline_engine_unreal.so"));
    RuntimeDependencies.Add(
        "$(PluginDir)/Source/GuidelineEngine/lib/libguideline_engine_unreal.so");

    PublicIncludePaths.AddRange(new string[] {});

    PrivateIncludePaths.AddRange(new string[] {});

    PublicDependencyModuleNames.AddRange(new string[] { "AudioMixer", "Core", "CoreUObject",
                                                        "Engine", "Projects", "RenderCore", "RHI",
                                                        "GuidelineShaders" });

    PrivateDependencyModuleNames.AddRange(new string[] {});

    DynamicallyLoadedModuleNames.AddRange(new string[] {});
  }
}
