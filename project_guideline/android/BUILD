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

load("@build_bazel_rules_android//android:rules.bzl", "android_binary")
load("//third_party/bazel_skylib/rules:common_settings.bzl", "bool_flag")

package(
    default_applicable_licenses = ["//:license"],
)

bool_flag(
    name = "guideline_classic_mode",
    build_setting_default = False,
)

config_setting(
    name = "use_guideline_classic_mode",
    flag_values = {
        ":guideline_classic_mode": "True",
    },
)

android_binary(
    name = "guideline_app",
    custom_package = "com.google.research.guideline",
    manifest = "AndroidManifest.xml",
    manifest_values = {
        "versionCode": "1",
        "versionName": "1.0",
        "applicationId": "com.google.research.guideline",
    },
    multidex = "native",
    nocompress_extensions = [
        ".tflite",
    ],
    deps = [
        "//project_guideline/android/java/com/google/research/guideline",
    ],
)
