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
load("//third_party/bazel_rules/rules_cc/cc:cc_test.bzl", "cc_test")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = [
        "//:internal",
    ],
)

cc_library(
    name = "guideline_detector",
    srcs = ["guideline_detector.cc"],
    hdrs = ["guideline_detector.h"],
    deps = [
        ":keypoint_extraction",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/util:embedded_file_toc",
        "//project_guideline/util:image",
        "//project_guideline/util:status",
        "//project_guideline/vision/models:depth_tflite_embed",
        "//project_guideline/vision/models:guideline_tflite_embed",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@eigen_archive//:eigen3",
        "@mediapipe//mediapipe/framework/formats:image",
        "@mediapipe//mediapipe/framework/formats:image_format_cc_proto",
        "@mediapipe//mediapipe/framework/formats:image_frame",
        "@mediapipe//mediapipe/framework/formats:image_frame_opencv",
        "@mediapipe//mediapipe/framework/port:opencv_core",
        "@mediapipe//mediapipe/tasks/cc/core:base_options",
        "@mediapipe//mediapipe/tasks/cc/vision/image_segmenter",
        "@mediapipe//mediapipe/tasks/cc/vision/image_segmenter:image_segmenter_result",
    ],
)

cc_test(
    name = "guideline_detector_test",
    srcs = ["guideline_detector_test.cc"],
    data = [
        "//project_guideline/vision/testdata:images",
    ],
    tags = ["not_run:arm"],
    deps = [
        ":guideline_detector",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/testing:images",
        "//project_guideline/testing:predicates",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:image",
        "//project_guideline/util:image_io",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/synchronization",
        "@com_google_googletest//:gtest_main",
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "keypoint_extraction",
    srcs = ["keypoint_extraction.cc"],
    hdrs = ["keypoint_extraction.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@eigen_archive//:eigen3",
        "@mediapipe//mediapipe/framework/port:opencv_core",
    ],
)
