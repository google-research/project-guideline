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
    name = "camera_model",
    hdrs = ["camera_model.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "camera_utils",
    srcs = ["camera_utils.cc"],
    hdrs = ["camera_utils.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":camera_model",
        "@eigen_archive//:eigen3",
    ],
)

cc_test(
    name = "camera_utils_test",
    srcs = ["camera_utils_test.cc"],
    deps = [
        ":camera_utils",
        ":cv_camera_model",
        "//project_guideline/testing:predicates",
        "@com_google_googletest//:gtest_main",
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "cv_camera_model",
    srcs = ["cv_camera_model.cc"],
    hdrs = ["cv_camera_model.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":camera_model",
        "@eigen_archive//:eigen3",
        "@mediapipe//mediapipe/framework/port:opencv_calib3d",
        "@mediapipe//mediapipe/framework/port:opencv_core",
        "@mediapipe//mediapipe/framework/port:opencv_imgproc",
    ],
)

cc_test(
    name = "cv_camera_model_test",
    srcs = ["cv_camera_model_test.cc"],
    deps = [
        ":cv_camera_model",
        "//project_guideline/testing:predicates",
        "@com_google_googletest//:gtest_main",
        "@eigen_archive//:eigen3",
    ],
)
