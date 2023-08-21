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
    name = "arcore_data_source",
    srcs = ["arcore_data_source.cc"],
    hdrs = ["arcore_data_source.h"],
    deps = [
        ":arcore_session",
        "//project_guideline/data:data_source",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "arcore_motion_tracker",
    srcs = ["arcore_motion_tracker.cc"],
    hdrs = ["arcore_motion_tracker.h"],
    deps = [
        ":arcore_session",
        "//project_guideline/camera:camera_model",
        "//project_guideline/camera:cv_camera_model",
        "//project_guideline/motion:motion_tracker",
        "//project_guideline/motion:tracking_feature",
        "//project_guideline/util:geometry",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "arcore_session",
    srcs = ["arcore_session.cc"],
    hdrs = ["arcore_session.h"],
    target_compatible_with = [
        "@platforms//os:android",
    ],
    deps = [
        "//project_guideline/util:image",
        "//project_guideline/util:image_yuv_conversion",
        "//project_guideline/util:status",
        "//project_guideline/util:transformation",
        "@arcore_android_sdk",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@eigen_archive//:eigen3",
    ],
)
