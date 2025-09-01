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
load("//third_party/protobuf/bazel:proto_library.bzl", "proto_library")
load("//tools/build_defs/proto/cpp:cc_proto_library.bzl", "cc_proto_library")

package(default_applicable_licenses = ["//:license"])

cc_library(
    name = "depth_alignment_calculator",
    srcs = ["depth_alignment_calculator.cc"],
    visibility = ["//visibility:private"],
    deps = [
        ":depth_alignment_calculator_options_cc_proto",
        "//project_guideline/camera:cv_camera_model",
        "//project_guideline/depth:depth_align_ransac",
        "//project_guideline/util:status",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@eigen_archive//:eigen3",
        "@mediapipe//mediapipe/framework:calculator_framework",
        "@mediapipe//mediapipe/framework:packet",
        "@mediapipe//mediapipe/framework/formats:image_frame",
        "@mediapipe//mediapipe/framework/formats:image_frame_opencv",
        "@mediapipe//mediapipe/framework/formats:landmark_cc_proto",
        "@mediapipe//mediapipe/framework/formats:matrix",
        "@mediapipe//mediapipe/framework/port:opencv_core",
        "@mediapipe//mediapipe/framework/port:ret_check",
    ],
    alwayslink = 1,
)

cc_test(
    name = "depth_alignment_calculator_test",
    srcs = ["depth_alignment_calculator_test.cc"],
    data = [
        "//project_guideline/vision/testdata:images",
    ],
    deps = [
        ":depth_alignment_calculator",
        "//project_guideline/camera:cv_camera_model",
        "//project_guideline/testing:images",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:image",
        "//project_guideline/util:image_io",
        "//project_guideline/util:mediapipe",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest_main",
        "@mediapipe//mediapipe/framework:calculator_framework",
        "@mediapipe//mediapipe/framework:calculator_runner",
        "@mediapipe//mediapipe/framework:packet",
        "@mediapipe//mediapipe/framework/formats:image_frame",
        "@mediapipe//mediapipe/framework/formats:landmark_cc_proto",
        "@mediapipe//mediapipe/framework/formats:matrix",
        "@mediapipe//mediapipe/framework/port:parse_text_proto",
    ],
)

proto_library(
    name = "depth_alignment_calculator_options_proto",
    srcs = ["depth_alignment_calculator_options.proto"],
    visibility = ["//visibility:private"],
)

cc_proto_library(
    name = "depth_alignment_calculator_options_cc_proto",
    visibility = ["//visibility:private"],
    deps = [":depth_alignment_calculator_options_proto"],
)

cc_library(
    name = "image_segmentation_calculator",
    srcs = ["image_segmentation_calculator.cc"],
    visibility = ["//visibility:private"],
    deps = [
        ":image_segmentation_calculator_options_cc_proto",
        "//project_guideline/util:status",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@mediapipe//mediapipe/framework:calculator_framework",
        "@mediapipe//mediapipe/framework/formats:image",
        "@mediapipe//mediapipe/framework/formats:image_frame",
        "@mediapipe//mediapipe/framework/port:ret_check",
        "@mediapipe//mediapipe/tasks/cc/vision/image_segmenter",
    ],
    alwayslink = 1,
)

cc_test(
    name = "image_segmentation_calculator_test",
    srcs = ["image_segmentation_calculator_test.cc"],
    data = [
        "//project_guideline/vision/models:depth.tflite",
        "//project_guideline/vision/models:guideline.tflite",
        "//project_guideline/vision/testdata:images",
    ],
    deps = [
        ":image_segmentation_calculator",
        "//project_guideline/testing:images",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:image",
        "//project_guideline/util:image_io",
        "//project_guideline/util:mediapipe",
        "//project_guideline/vision/models:guideline_tflite_embed",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest_main",
        "@mediapipe//mediapipe/framework:calculator_framework",
        "@mediapipe//mediapipe/framework:calculator_runner",
        "@mediapipe//mediapipe/framework:packet",
        "@mediapipe//mediapipe/framework/formats:image_frame",
        "@mediapipe//mediapipe/framework/port:parse_text_proto",
    ],
)

proto_library(
    name = "image_segmentation_calculator_options_proto",
    srcs = ["image_segmentation_calculator_options.proto"],
    visibility = ["//visibility:private"],
)

cc_proto_library(
    name = "image_segmentation_calculator_options_cc_proto",
    visibility = ["//visibility:private"],
    deps = [":image_segmentation_calculator_options_proto"],
)

cc_library(
    name = "occupancy_map_calculator",
    srcs = ["occupancy_map_calculator.cc"],
    visibility = ["//visibility:private"],
    deps = [
        ":occupancy_map_calculator_options_cc_proto",
        "//project_guideline/camera:cv_camera_model",
        "//project_guideline/depth:point_cloud_util",
        "//project_guideline/environment:obstacle_utils",
        "//project_guideline/util:image",
        "//project_guideline/util:status",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@eigen_archive//:eigen3",
        "@mediapipe//mediapipe/framework:calculator_framework",
        "@mediapipe//mediapipe/framework/formats:image",
        "@mediapipe//mediapipe/framework/formats:image_frame",
        "@mediapipe//mediapipe/framework/formats:image_frame_opencv",
        "@mediapipe//mediapipe/framework/formats:matrix",
        "@mediapipe//mediapipe/framework/port:ret_check",
    ],
    alwayslink = 1,
)

cc_test(
    name = "occupancy_map_calculator_test",
    srcs = ["occupancy_map_calculator_test.cc"],
    data = [
        "//project_guideline/vision/models:depth.tflite",
        "//project_guideline/vision/models:guideline.tflite",
        "//project_guideline/vision/testdata:images",
    ],
    deps = [
        ":occupancy_map_calculator",
        "//project_guideline/camera:cv_camera_model",
        "//project_guideline/testing:images",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:geometry",
        "//project_guideline/util:image",
        "//project_guideline/util:image_io",
        "//project_guideline/util:mediapipe",
        "//project_guideline/util:transformation",
        "//project_guideline/vision/models:guideline_tflite_embed",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest_main",
        "@mediapipe//mediapipe/framework:calculator_framework",
        "@mediapipe//mediapipe/framework:calculator_runner",
        "@mediapipe//mediapipe/framework:packet",
        "@mediapipe//mediapipe/framework/formats:image_frame",
        "@mediapipe//mediapipe/framework/formats:matrix",
        "@mediapipe//mediapipe/framework/port:parse_text_proto",
    ],
)

cc_test(
    name = "obstacle_detection_graph_test",
    srcs = ["obstacle_detection_graph_test.cc"],
    data = [
        "//project_guideline/vision/models:depth.tflite",
        "//project_guideline/vision/testdata:images",
    ],
    deps = [
        ":depth_alignment_calculator",
        ":image_segmentation_calculator",
        ":occupancy_map_calculator",
        "//project_guideline/camera:cv_camera_model",
        "//project_guideline/testing:images",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:geometry",
        "//project_guideline/util:image",
        "//project_guideline/util:image_io",
        "//project_guideline/util:mediapipe",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest_main",
        "@mediapipe//mediapipe/framework:calculator_framework",
        "@mediapipe//mediapipe/framework:calculator_runner",
        "@mediapipe//mediapipe/framework:packet",
        "@mediapipe//mediapipe/framework/formats:image_frame",
        "@mediapipe//mediapipe/framework/formats:landmark_cc_proto",
        "@mediapipe//mediapipe/framework/formats:matrix",
        "@mediapipe//mediapipe/framework/port:parse_text_proto",
    ],
)

proto_library(
    name = "occupancy_map_calculator_options_proto",
    srcs = ["occupancy_map_calculator_options.proto"],
    visibility = ["//visibility:private"],
)

cc_proto_library(
    name = "occupancy_map_calculator_options_cc_proto",
    visibility = ["//visibility:private"],
    deps = [":occupancy_map_calculator_options_proto"],
)
