# Copyright 2023 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("@build_bazel_rules_android//android:rules.bzl", "android_library")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = [
        "//:internal",
    ],
)

android_library(
    name = "guidance",
    srcs = glob(["*.java"]),
    exports_manifest = 1,
    manifest = "AndroidManifest.xml",
    resource_files = glob(["res/**"]),
    deps = [
        "//project_guideline/android/java/com/google/research/guideline/inject",
        "//project_guideline/android/java/com/google/research/guideline/util/permissions",
        "//project_guideline/android/java/com/google/research/guideline/util/ui",
        "//third_party/hilt:android_entry_point",
        "@maven//:androidx_appcompat_appcompat",
        "@maven//:com_google_code_findbugs_jsr305",
        "@maven//:javax_inject_javax_inject",
    ],
)
