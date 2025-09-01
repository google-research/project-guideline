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
    name = "guideline_logger",
    srcs = ["guideline_logger.cc"],
    hdrs = ["guideline_logger.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//project_guideline/proto:guideline_log_cc_proto",
        "//project_guideline/util:image",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_test(
    name = "guideline_logger_test",
    srcs = ["guideline_logger_test.cc"],
    deps = [
        ":guideline_logger",
        ":noop_guideline_logger",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "file_guideline_logger",
    srcs = ["file_guideline_logger.cc"],
    hdrs = ["file_guideline_logger.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":guideline_logger",
        "//project_guideline/proto:guideline_log_cc_proto",
        "//project_guideline/util:image",
        "//project_guideline/util:image_io",
        "//third_party/protobuf",
        "//third_party/protobuf:io",
        "//third_party/protobuf:time_util",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "noop_guideline_logger",
    hdrs = ["noop_guideline_logger.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":guideline_logger",
        "//project_guideline/proto:guideline_log_cc_proto",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "debug_signal",
    hdrs = ["debug_signal.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "debug_signal_provider",
    srcs = ["debug_signal_provider.cc"],
    hdrs = ["debug_signal_provider.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":debug_signal",
        "//project_guideline/camera:camera_model",
        "//project_guideline/environment:control_signal",
        "//project_guideline/motion:tracking_feature",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/util:image",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@eigen_archive//:eigen3",
    ],
)
