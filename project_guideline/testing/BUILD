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
    default_testonly = 1,
    default_visibility = [
        "//:internal",
    ],
)

cc_library(
    name = "gl_libs",
    visibility = ["//visibility:public"],
    deps = select({
        "//:swiftshader": [
            "@swiftshader//:EGL",
            "@swiftshader//:GLESv2",
        ],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "images",
    srcs = ["images.cc"],
    hdrs = ["images.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//project_guideline/util:image",
        "@com_google_absl//absl/log:check",
        "@mediapipe//mediapipe/framework/port:opencv_core",
    ],
)

cc_library(
    name = "predicates",
    hdrs = ["predicates.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//project_guideline/util:transformation",
        "@com_google_googletest//:gtest",
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "simulated_clock",
    hdrs = ["simulated_clock.h"],
    deps = [
        "//project_guideline/util:clock",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "status_matchers",
    hdrs = ["status_matchers.h"],
    deps = [
        "//project_guideline/util:status",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_googletest//:gtest",
    ],
)
