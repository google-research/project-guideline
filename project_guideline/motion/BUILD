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
    name = "motion_tracker",
    srcs = ["motion_tracker.cc"],
    hdrs = ["motion_tracker.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":tracking_feature",
        "//project_guideline/camera:camera_model",
        "//project_guideline/util:image",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "passthrough_motion_tracker",
    srcs = ["passthrough_motion_tracker.cc"],
    hdrs = ["passthrough_motion_tracker.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//project_guideline/camera:camera_model",
        "//project_guideline/logging:guideline_logger",
        "//project_guideline/motion:motion_tracker",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "tracking_feature",
    hdrs = ["tracking_feature.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@eigen_archive//:eigen3",
    ],
)
