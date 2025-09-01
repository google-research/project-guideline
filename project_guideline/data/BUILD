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
    name = "data_source",
    hdrs = ["data_source.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//project_guideline/util:image",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "passthrough_data_source",
    srcs = ["passthrough_data_source.cc"],
    hdrs = ["passthrough_data_source.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":data_source",
        "//project_guideline/util:image",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
    ],
)
