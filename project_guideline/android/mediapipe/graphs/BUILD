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

load(
    "//research/drishti/framework/tool:drishti_graph.bzl",
    "drishti_binary_graph",
)
load("//third_party/bazel_rules/rules_cc/cc:cc_library.bzl", "cc_library")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = [
        "//:internal",
    ],
)

filegroup(
    name = "mobile_graphs",
    srcs = [
        ":guideline_classic_graph.binarypb",
    ],
)

drishti_binary_graph(
    name = "guideline_classic_graph",
    graph = "guideline_classic_graph.pbtxt",
    output_name = "guideline_classic_graph.binarypb",
    deps = [
        ":mobile_calculators",
    ],
)

cc_library(
    name = "mobile_calculators",
    deps = [
        "//project_guideline/android/mediapipe/calculators:bool_to_int_calculator",
        "//project_guideline/android/mediapipe/calculators:classic_guidance_calculator",
        "//project_guideline/android/mediapipe/calculators:classic_render_data_calculator",
        "//project_guideline/android/mediapipe/calculators:line_mask_tensors_to_line_calculator",
        "//project_guideline/android/mediapipe/calculators:mask_overlay_calculator",
        "@mediapipe//mediapipe/calculators/core:flow_limiter_calculator",
        "@mediapipe//mediapipe/calculators/core:gate_calculator",
        "@mediapipe//mediapipe/calculators/core:mux_calculator",
        "@mediapipe//mediapipe/calculators/image:color_convert_calculator",
        "@mediapipe//mediapipe/calculators/image:image_cropping_calculator",
        "@mediapipe//mediapipe/calculators/tensor:image_to_tensor_calculator",
        "@mediapipe//mediapipe/calculators/tensor:inference_calculator",
        "@mediapipe//mediapipe/calculators/util:annotation_overlay_calculator",
        "@mediapipe//mediapipe/calculators/util:renderer_calculator",
        "@mediapipe//mediapipe/calculators/util:to_image_calculator",
        "@mediapipe//mediapipe/gpu:gpu_buffer_to_image_frame_calculator",
        "@mediapipe//mediapipe/gpu:image_frame_to_gpu_buffer_calculator",
    ],
)
