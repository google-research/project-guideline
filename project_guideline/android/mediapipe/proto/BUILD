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
    name = "classic_guidance_proto",
    srcs = ["classic_guidance.proto"],
    deps = [":line_detection_proto"],
)

cc_proto_library(
    name = "classic_guidance_cc_proto",
    deps = [":classic_guidance_proto"],
)

java_lite_proto_library(
    name = "classic_guidance_java_proto_lite",
    deps = [":classic_guidance_proto"],
)

proto_library(
    name = "line_detection_proto",
    srcs = ["line_detection.proto"],
)

cc_proto_library(
    name = "line_detection_cc_proto",
    deps = [":line_detection_proto"],
)

java_lite_proto_library(
    name = "line_detection_java_proto_lite",
    deps = [":line_detection_proto"],
)
