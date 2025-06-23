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
    name = "averaging_filter",
    hdrs = ["averaging_filter.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/time",
    ],
)

cc_test(
    name = "averaging_filter_test",
    srcs = ["averaging_filter_test.cc"],
    deps = [
        ":averaging_filter",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "biquad_filter",
    srcs = ["biquad_filter.cc"],
    hdrs = ["biquad_filter.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/time",
        "@eigen_archive//:eigen3",
    ],
)

cc_test(
    name = "biquad_filter_test",
    srcs = ["biquad_filter_test.cc"],
    deps = [
        ":biquad_filter",
        "@com_google_absl//absl/random",
        "@com_google_absl//absl/random:distributions",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "clock",
    srcs = ["clock.cc"],
    hdrs = ["clock.h"],
    deps = [
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "cubic_spline",
    srcs = ["cubic_spline.cc"],
    hdrs = ["cubic_spline.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@eigen_archive//:eigen3",
    ],
)

cc_test(
    name = "cubic_spline_test",
    srcs = ["cubic_spline_test.cc"],
    deps = [
        ":cubic_spline",
        "@com_google_googletest//:gtest_main",
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "embedded_file_toc",
    hdrs = ["embedded_file_toc.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "file",
    srcs = ["file.cc"],
    hdrs = ["file.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "geometry",
    srcs = ["geometry.cc"],
    hdrs = ["geometry.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":transformation",
        "@eigen_archive//:eigen3",
    ],
)

cc_test(
    name = "geometry_test",
    srcs = ["geometry_test.cc"],
    deps = [
        ":geometry",
        "//project_guideline/testing:predicates",
        "@com_google_googletest//:gtest_main",
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "hit_test_util",
    srcs = ["hit_test_util.cc"],
    hdrs = ["hit_test_util.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//project_guideline/camera:camera_model",
        "//project_guideline/util:transformation",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@eigen_archive//:eigen3",
    ],
)

cc_test(
    name = "hit_test_util_test",
    srcs = ["hit_test_util_test.cc"],
    deps = [
        ":hit_test_util",
        "//project_guideline/camera:cv_camera_model",
        "//project_guideline/testing:predicates",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:geometry",
        "//project_guideline/util:transformation",
        "@com_google_googletest//:gtest_main",
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "image",
    srcs = ["image.cc"],
    hdrs = ["image.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log:check",
        "@mediapipe//mediapipe/framework/port:opencv_core",
        "@mediapipe//mediapipe/framework/port:opencv_imgproc",
    ],
)

cc_library(
    name = "image_io",
    srcs = ["image_io.cc"],
    hdrs = ["image_io.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":image",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@mediapipe//mediapipe/framework/port:opencv_core",
        "@mediapipe//mediapipe/framework/port:opencv_imgcodecs",
        "@mediapipe//mediapipe/framework/port:opencv_imgproc",
    ],
)

cc_library(
    name = "image_yuv_conversion",
    srcs = ["image_yuv_conversion.cc"],
    hdrs = ["image_yuv_conversion.h"],
    deps = [
        ":image",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status:statusor",
        "@libyuv",
        "@mediapipe//mediapipe/framework/port:opencv_core",
    ],
)

cc_test(
    name = "image_yuv_conversion_test",
    srcs = ["image_yuv_conversion_test.cc"],
    data = ["//project_guideline/util/testdata:yuv_images"],
    deps = [
        ":file",
        ":image",
        ":image_io",
        ":image_yuv_conversion",
        "//project_guideline/testing:status_matchers",
        "@com_google_googletest//:gtest_main",
        "@mediapipe//mediapipe/framework/port:opencv_core",
    ],
)

cc_library(
    name = "lerp",
    hdrs = ["lerp.h"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "lerp_test",
    srcs = ["lerp_test.cc"],
    deps = [
        ":lerp",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "math",
    hdrs = ["math.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "math_test",
    srcs = ["math_test.cc"],
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "mediapipe",
    srcs = ["mediapipe.cc"],
    hdrs = ["mediapipe.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//project_guideline/util:image",
        "//project_guideline/util:image_io",
        "//project_guideline/util:status",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status:statusor",
        "@mediapipe//mediapipe/framework/formats:image_frame",
        "@mediapipe//mediapipe/framework/formats:image_frame_opencv",
    ],
)

cc_library(
    name = "status",
    hdrs = ["status.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "transformation",
    hdrs = ["transformation.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@eigen_archive//:eigen3",
    ],
)

cc_test(
    name = "transformation_test",
    srcs = ["transformation_test.cc"],
    deps = [
        ":transformation",
        "//project_guideline/testing:predicates",
        "@com_google_googletest//:gtest_main",
        "@eigen_archive//:eigen3",
    ],
)

cc_library(
    name = "windowed_value_latch",
    hdrs = ["windowed_value_latch.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_test(
    name = "windowed_value_latch_test",
    srcs = ["windowed_value_latch_test.cc"],
    deps = [
        ":windowed_value_latch",
        "@com_google_googletest//:gtest_main",
    ],
)
