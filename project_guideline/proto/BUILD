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

load("//third_party/protobuf/bazel:java_lite_proto_library.bzl", "java_lite_proto_library")
load("//third_party/protobuf/bazel:proto_library.bzl", "proto_library")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = [
        "//:internal",
    ],
)

proto_library(
    name = "control_signal_proto",
    srcs = ["control_signal.proto"],
    visibility = [
        "//visibility:public",
    ],
)

cc_proto_library(
    name = "control_signal_cc_proto",
    visibility = [
        "//visibility:public",
    ],
    deps = [":control_signal_proto"],
)

java_lite_proto_library(
    name = "control_signal_java_proto_lite",
    visibility = [
        "//visibility:public",
    ],
    deps = [":control_signal_proto"],
)

proto_library(
    name = "control_signal_data_proto",
    srcs = ["control_signal_data.proto"],
    visibility = [
        "//visibility:public",
    ],
)

cc_proto_library(
    name = "control_signal_data_cc_proto",
    visibility = [
        "//visibility:public",
    ],
    deps = [":control_signal_data_proto"],
)

proto_library(
    name = "guideline_engine_config_proto",
    srcs = ["guideline_engine_config.proto"],
    deps = [
        "@com_google_protobuf//:duration_proto",
    ],
)

cc_proto_library(
    name = "guideline_engine_config_cc_proto",
    visibility = [
        "//visibility:public",
    ],
    deps = [":guideline_engine_config_proto"],
)

java_lite_proto_library(
    name = "guideline_engine_config_java_proto_lite",
    visibility = [
        "//visibility:public",
    ],
    deps = [":guideline_engine_config_proto"],
)

proto_library(
    name = "guideline_log_proto",
    srcs = ["guideline_log.proto"],
    deps = [
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "guideline_log_cc_proto",
    visibility = [
        "//visibility:public",
    ],
    deps = [":guideline_log_proto"],
)
