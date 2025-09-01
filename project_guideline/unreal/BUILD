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

load("//third_party/bazel_rules/rules_cc/cc:cc_binary.bzl", "cc_binary")
load("//third_party/bazel_rules/rules_cc/cc:cc_library.bzl", "cc_library")
load("//third_party/bazel_rules/rules_cc/cc:cc_test.bzl", "cc_test")

cc_binary(
    name = "libguideline_engine_unreal.so",
    linkopts = [
        # Ensure that all required symbols are statically linked into the shared library.
        "-Wl,--no-allow-shlib-undefined",
        "-Wl,-z,defs",
    ],
    linkshared = 1,
    linkstatic = 1,
    deps = [":unreal_plugin"],
)

cc_library(
    name = "unreal_plugin",
    srcs = ["unreal_plugin_impl.cc"],
    hdrs = [
        "unreal_plugin.h",
        "unreal_plugin_impl.h",
    ],
    deps = [
        ":unreal_audio_output_stream",
        "//project_guideline/camera:camera_model",
        "//project_guideline/camera:camera_utils",
        "//project_guideline/camera:cv_camera_model",
        "//project_guideline/data:passthrough_data_source",
        "//project_guideline/engine:guideline_engine",
        "//project_guideline/environment:control_signal",
        "//project_guideline/logging:file_guideline_logger",
        "//project_guideline/logging:guideline_logger",
        "//project_guideline/logging:noop_guideline_logger",
        "//project_guideline/motion:passthrough_motion_tracker",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/util:geometry",
        "//project_guideline/util:image",
        "//project_guideline/util:lerp",
        "//project_guideline/util:status",
        "//project_guideline/util:transformation",
        "//third_party/protobuf:duration_cc_proto",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@eigen_archive//:eigen3",
        "@mediapipe//mediapipe/framework/port:opencv_core",
        "@mediapipe//mediapipe/framework/port:opencv_imgproc",
    ],
    alwayslink = 1,
)

cc_test(
    name = "unreal_plugin_test",
    srcs = ["unreal_plugin_test.cc"],
    deps = [
        ":unreal_plugin",
        "//project_guideline/camera:camera_model",
        "//project_guideline/motion:motion_tracker",
        "//project_guideline/proto:guideline_log_cc_proto",
        "//project_guideline/testing:predicates",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "unreal_audio_output_stream",
    srcs = ["unreal_audio_output_stream.cc"],
    hdrs = [
        "unreal_audio_output_stream.h",
    ],
    deps = [
        "//project_guideline/audio:audio_output_stream",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
    ],
)
