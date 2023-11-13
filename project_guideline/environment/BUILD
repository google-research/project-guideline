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

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = [
        "//:internal",
    ],
)

cc_library(
    name = "control_signal",
    hdrs = ["control_signal.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//project_guideline/proto:control_signal_data_cc_proto",
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "environment",
    hdrs = ["environment.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":guideline_aggregator",
        ":object",
        ":obstacle",
        ":occupancy_map",
        ":point_cloud",
        "//project_guideline/util:status",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "guidance_system",
    srcs = ["guidance_system.cc"],
    hdrs = ["guidance_system.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//project_guideline/camera:camera_model",
        "//project_guideline/depth:depth_align_ransac",
        "//project_guideline/depth:point_cloud_util",
        "//project_guideline/environment",
        "//project_guideline/environment:control_signal",
        "//project_guideline/environment:obstacle",
        "//project_guideline/environment:path_planning",
        "//project_guideline/logging:guideline_logger",
        "//project_guideline/motion:motion_tracker",
        "//project_guideline/motion:tracking_feature",
        "//project_guideline/util:geometry",
        "//project_guideline/util:hit_test_util",
        "//project_guideline/util:image",
        "//project_guideline/util:transformation",
        "//third_party/protobuf:time_util",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
        "@eigen_archive//:eigen3",
        "@mediapipe//mediapipe/framework/port:opencv_core",
    ],
)

cc_test(
    name = "guidance_system_test",
    srcs = ["guidance_system_test.cc"],
    deps = [
        ":guidance_system",
        "//project_guideline/camera:cv_camera_model",
        "//project_guideline/environment",
        "//project_guideline/environment:control_signal",
        "//project_guideline/environment:human_representation",
        "//project_guideline/environment:path_planning",
        "//project_guideline/logging:file_guideline_logger",
        "//project_guideline/motion:tracking_feature",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/util:geometry",
        "//project_guideline/util:image",
        "//project_guideline/util:status",
        "//project_guideline/util:transformation",
        "//third_party/protobuf:duration_cc_proto",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest_main",
        "@mediapipe//mediapipe/framework/port:opencv_core",
    ],
)

cc_library(
    name = "guideline_aggregator",
    srcs = ["guideline_aggregator.cc"],
    hdrs = ["guideline_aggregator.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/util:cubic_spline",
        "//project_guideline/util:geometry",
        "//project_guideline/util:hit_test_util",
        "//project_guideline/util:math",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@eigen_archive//:eigen3",
    ],
)

cc_test(
    name = "guideline_aggregator_test",
    srcs = ["guideline_aggregator_test.cc"],
    stamp = 1,
    deps = [
        ":guideline_aggregator",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/testing:predicates",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:geometry",
        "//project_guideline/util:hit_test_util",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/random",
        "@com_google_absl//absl/random:distributions",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "human_representation",
    srcs = ["human_representation.cc"],
    hdrs = ["human_representation.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":object",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/util:status",
        "//project_guideline/util:transformation",
        "//third_party/protobuf:time_util",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
)

cc_test(
    name = "human_representation_test",
    srcs = ["human_representation_test.cc"],
    deps = [
        ":human_representation",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/testing:status_matchers",
        "//third_party/protobuf:time_util",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "object",
    hdrs = ["object.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/status:statusor",
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "obstacle",
    hdrs = ["obstacle.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":object",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "obstacle_utils",
    srcs = ["obstacle_utils.cc"],
    hdrs = ["obstacle_utils.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//project_guideline/util:geometry",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/container:flat_hash_map",
        "@eigen_archive//:eigen3",
    ],
)

cc_test(
    name = "obstacle_utils_test",
    srcs = ["obstacle_utils_test.cc"],
    deps = [
        ":obstacle_utils",
        "//project_guideline/util:transformation",
        "@com_google_googletest//:gtest_main",
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "occupancy_map",
    srcs = ["occupancy_map.cc"],
    hdrs = ["occupancy_map.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":obstacle_utils",
        "//project_guideline/depth:point_cloud_util",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@eigen_archive//:eigen3",
    ],
)

cc_test(
    name = "occupancy_map_test",
    srcs = ["occupancy_map_test.cc"],
    deps = [
        ":occupancy_map",
        "//project_guideline/depth:point_cloud_util",
        "//project_guideline/util:transformation",
        "@com_google_googletest//:gtest_main",
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "path_planning",
    srcs = ["path_planning.cc"],
    hdrs = ["path_planning.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":obstacle",
        "//project_guideline/environment:control_signal",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/util:geometry",
        "//project_guideline/util:windowed_value_latch",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
)

cc_test(
    name = "path_planning_test",
    srcs = ["path_planning_test.cc"],
    deps = [
        ":path_planning",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:geometry",
        "//project_guideline/util:transformation",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "point_cloud",
    srcs = ["point_cloud.cc"],
    hdrs = ["point_cloud.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//project_guideline/camera:camera_model",
        "//project_guideline/depth:point_cloud_util",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/util:image",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_test(
    name = "point_cloud_test",
    srcs = ["point_cloud_test.cc"],
    deps = [
        ":point_cloud",
        "//project_guideline/camera:cv_camera_model",
        "//project_guideline/depth:point_cloud_util",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:geometry",
        "//project_guideline/util:image",
        "//project_guideline/util:transformation",
        "@com_google_googletest//:gtest_main",
        "@eigen_archive//:eigen3",
    ],
)
