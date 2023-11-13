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

load("@bazel_skylib//rules:common_settings.bzl", "string_flag")
load("@dagger//:workspace_defs.bzl", "dagger_android_rules", "hilt_android_rules")
load("@rules_license//rules:license.bzl", "license")

licenses(["notice"])

exports_files(["LICENSE"])

license(
    name = "license",
    package_name = "project_guideline",
)

package_group(
    name = "internal",
    packages = ["//project_guideline/..."],
)

# Flag used to control whether to use OpenGL libraries from the system or to
# use the swiftshader software implementation. This is intended for running
# tests and utilities using swiftshader on machines that do not have a GPU.
# Example: bazelisk test --define MEDIAPIPE_DISABLE_GPU=1 --//:gl=swiftshader \
#                //project_guideline/visualization:all
string_flag(
    name = "gl",
    build_setting_default = "system",
    values = [
        "system",
        "swiftshader",
    ],
)

config_setting(
    name = "swiftshader",
    flag_values = {
        ":gl": "swiftshader",
    },
)

platform(
    name = "arm64-v8a",
    constraint_values = [
        "@platforms//os:android",
        "@platforms//cpu:arm64",
    ],
)

dagger_android_rules()

hilt_android_rules()
