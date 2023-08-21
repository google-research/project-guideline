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
    name = "camera_feed_renderer",
    srcs = ["camera_feed_renderer.cc"],
    hdrs = ["camera_feed_renderer.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":gl_util",
        "//project_guideline/util:image",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
        "@eigen_archive//:eigen3",
    ],
)

cc_test(
    name = "camera_feed_renderer_test",
    srcs = ["camera_feed_renderer_test.cc"],
    data = ["//project_guideline/util/testdata:landscape_image"],
    deps = [
        ":camera_feed_renderer",
        ":render_context",
        "//project_guideline/testing:gl_libs",
        "//project_guideline/testing:images",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:image",
        "//project_guideline/util:image_io",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "environment_map_renderer",
    srcs = ["environment_map_renderer.cc"],
    hdrs = ["environment_map_renderer.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//project_guideline/environment:control_signal",
        "//project_guideline/util:geometry",
        "//project_guideline/util:transformation",
        "//project_guideline/visualization:gl_util",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/synchronization",
        "@eigen_archive//:eigen3",
    ],
)

cc_test(
    name = "environment_map_renderer_test",
    srcs = ["environment_map_renderer_test.cc"],
    data = [
        "//project_guideline/visualization/testdata:map_goldens",
    ],
    deps = [
        ":environment_map_renderer",
        ":render_context",
        "//project_guideline/environment:control_signal",
        "//project_guideline/testing:gl_libs",
        "//project_guideline/testing:images",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:geometry",
        "//project_guideline/util:image",
        "//project_guideline/util:image_io",
        "//project_guideline/util:transformation",
        "@com_google_googletest//:gtest_main",
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "gl_util",
    srcs = ["gl_util.cc"],
    hdrs = ["gl_util.h"],
    visibility = ["//visibility:public"],
    deps = ["@com_google_absl//absl/log:check"],
)

cc_library(
    name = "render_context",
    srcs = ["render_context.cc"],
    hdrs = ["render_context.h"],
    deps = [
        ":gl_util",
        "//project_guideline/util:image",
        "//project_guideline/util:status",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@mediapipe//mediapipe/framework/port:opencv_core",
        "@mediapipe//mediapipe/framework/port:opencv_highgui",
        "@mediapipe//mediapipe/framework/port:opencv_imgcodecs",
        "@mediapipe//mediapipe/framework/port:opencv_imgproc",
    ],
)
