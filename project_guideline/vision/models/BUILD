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

load("//project_guideline/bazel:gl_cc_embed_data.bzl", "gl_cc_embed_data")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = [
        "//:internal",
    ],
)

exports_files([
    "depth.tflite",
    "guideline.tflite",
    "guideline_f32input_singlechannel.tflite",
])

gl_cc_embed_data(
    name = "depth_tflite_embed",
    srcs = ["depth.tflite"],
    namespace = "guideline::vision",
)

gl_cc_embed_data(
    name = "guideline_tflite_embed",
    srcs = ["guideline.tflite"],
    namespace = "guideline::vision",
)
