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
    name = "depth_align_ransac",
    srcs = ["depth_align_ransac.cc"],
    hdrs = ["depth_align_ransac.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//project_guideline/camera:camera_model",
        "//project_guideline/motion:tracking_feature",
        "//project_guideline/util:image",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@eigen_archive//:eigen3",
        "@mediapipe//mediapipe/framework/port:opencv_calib3d",
        "@mediapipe//mediapipe/framework/port:opencv_core",
    ],
)

cc_test(
    name = "depth_align_ransac_test",
    srcs = ["depth_align_ransac_test.cc"],
    deps = [
        ":depth_align_ransac",
        "//project_guideline/camera:cv_camera_model",
        "//project_guideline/motion:tracking_feature",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:image",
        "@com_google_googletest//:gtest_main",
        "@eigen_archive//:eigen3",
        "@mediapipe//mediapipe/framework/port:opencv_core",
    ],
)

cc_library(
    name = "point_cloud_util",
    srcs = ["point_cloud_util.cc"],
    hdrs = ["point_cloud_util.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//project_guideline/camera:camera_model",
        "//project_guideline/util:image",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@eigen_archive//:eigen3",
    ],
)

cc_test(
    name = "point_cloud_util_test",
    srcs = ["point_cloud_util_test.cc"],
    deps = [
        ":point_cloud_util",
        "//project_guideline/camera:cv_camera_model",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:geometry",
        "//project_guideline/util:image",
        "//project_guideline/util:transformation",
        "@com_google_googletest//:gtest_main",
        "@mediapipe//mediapipe/framework/port:opencv_core",
    ],
)
