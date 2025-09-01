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

load("//third_party/bazel_rules/rules_cc/cc:cc_library.bzl", "cc_library")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = [
        "//:internal",
    ],
)

cc_library(
    name = "guideline_engine",
    srcs = ["guideline_engine.cc"],
    hdrs = ["guideline_engine.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//project_guideline/audio:audio_output_stream",
        "//project_guideline/audio:audio_system",
        "//project_guideline/camera:camera_model",
        "//project_guideline/data:data_source",
        "//project_guideline/environment",
        "//project_guideline/environment:guidance_system",
        "//project_guideline/environment:guideline_aggregator",
        "//project_guideline/environment:human_representation",
        "//project_guideline/environment:occupancy_map",
        "//project_guideline/environment:path_planning",
        "//project_guideline/environment:point_cloud",
        "//project_guideline/logging:guideline_logger",
        "//project_guideline/motion:motion_tracker",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/util:status",
        "//project_guideline/vision:guideline_detector",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)
