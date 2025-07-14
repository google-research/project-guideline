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
    name = "bool_to_int_calculator",
    srcs = ["bool_to_int_calculator.cc"],
    deps = [
        "@com_google_absl//absl/status",
        "@mediapipe//mediapipe/framework:calculator_framework",
        "@mediapipe//mediapipe/framework/port:ret_check",
    ],
)

cc_library(
    name = "classic_guidance_calculator",
    srcs = ["classic_guidance_calculator.cc"],
    deps = [
        "//project_guideline/android/mediapipe/proto:classic_guidance_cc_proto",
        "//project_guideline/android/mediapipe/proto:line_detection_cc_proto",
        "//project_guideline/util:averaging_filter",
        "//project_guideline/util:biquad_filter",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/time",
        "@mediapipe//mediapipe/framework:calculator_framework",
        "@mediapipe//mediapipe/framework/port:ret_check",
    ],
)

cc_library(
    name = "classic_render_data_calculator",
    srcs = ["classic_render_data_calculator.cc"],
    deps = [
        "//project_guideline/android/mediapipe/proto:classic_guidance_cc_proto",
        "//project_guideline/android/mediapipe/proto:line_detection_cc_proto",
        "@com_google_absl//absl/status",
        "@mediapipe//mediapipe/framework:calculator_framework",
        "@mediapipe//mediapipe/framework/port:ret_check",
        "@mediapipe//mediapipe/util:render_data_cc_proto",
    ],
)

cc_library(
    name = "line_mask_tensors_to_line_calculator",
    srcs = ["line_mask_tensors_to_line_calculator.cc"],
    deps = [
        "//project_guideline/android/mediapipe/proto:line_detection_cc_proto",
        "//project_guideline/util:hough_transform",
        "//project_guideline/util:lerp",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@eigen_archive//:eigen3",
        "@mediapipe//mediapipe/framework:calculator_framework",
        "@mediapipe//mediapipe/framework/formats:image",
        "@mediapipe//mediapipe/framework/formats:tensor",
        "@mediapipe//mediapipe/framework/port:opencv_core",
        "@mediapipe//mediapipe/framework/port:ret_check",
    ],
)

cc_library(
    name = "mask_overlay_calculator",
    srcs = ["mask_overlay_calculator.cc"],
    deps = [
        "//project_guideline/android/mediapipe/proto:line_detection_cc_proto",
        "//project_guideline/util:status",
        "//project_guideline/visualization:gl_util",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@mediapipe//mediapipe/framework:calculator_framework",
        "@mediapipe//mediapipe/framework/formats:tensor",
        "@mediapipe//mediapipe/framework/port:ret_check",
        "@mediapipe//mediapipe/gpu:gl_calculator_helper",
        "@mediapipe//mediapipe/gpu:gl_simple_shaders",
        "@mediapipe//mediapipe/gpu:gpu_buffer",
        "@mediapipe//mediapipe/gpu:shader_util",
    ],
)
