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

ALL_OGG_ASSETS = [
    "legacy_steering",
    "legacy_steering_20211014",
    "legacy_v4_1_steering",
    "legacy_v4_1_stop",
    "legacy_v4_1_warning",
    "legacy_v4_2_steering",
    "legacy_v4_2_stop",
    "legacy_v4_2_warning",
    "legacy_warning",
    "low_battery",
    "loading",
    "midleft",
    "midright",
    "obstacle",
    "ready",
    "sharpleft",
    "sharpright",
    "slightleft",
    "slightright",
    "steering_off_course_impulse_tick",
    "steering_on_course_impulse_tick",
    "stopstopstop",
    "straight",
    "warning_pitch_shift_2x",
]

[
    gl_cc_embed_data(
        name = "%s_embed" % ogg_asset,
        srcs = [
            "%s.ogg" % ogg_asset,
        ],
        namespace = "guideline::audio",
    )
    for ogg_asset in ALL_OGG_ASSETS
]
