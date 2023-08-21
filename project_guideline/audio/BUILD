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
    name = "audio_buffer",
    hdrs = ["audio_buffer.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "audio_output_stream",
    hdrs = ["audio_output_stream.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "audio_stream_renderer",
    srcs = ["audio_stream_renderer.cc"],
    hdrs = ["audio_stream_renderer.h"],
    deps = [
        ":audio_buffer",
        ":resampler",
        ":vorbis_stream_decoder",
        "//project_guideline/util:status",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "audio_system",
    srcs = ["audio_system.cc"],
    hdrs = ["audio_system.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":audio_output_stream",
        ":demo_resonant_sound_pack",
        ":legacy_sound_pack",
        ":sound_pack",
        ":sound_player",
        ":vorbis_stream_encoder",
        "//project_guideline/audio/assets:loading_embed",
        "//project_guideline/audio/assets:low_battery_embed",
        "//project_guideline/audio/assets:ready_embed",
        "//project_guideline/audio/assets:stopstopstop_embed",
        "//project_guideline/environment:control_signal",
        "//project_guideline/logging:guideline_logger",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/util:status",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@resonance_audio",
    ],
)

cc_test(
    name = "audio_system_test",
    srcs = ["audio_system_test.cc"],
    deps = [
        ":audio_buffer",
        ":audio_output_stream",
        ":audio_system",
        ":vorbis_stream_decoder",
        "//project_guideline/environment:control_signal",
        "//project_guideline/logging:noop_guideline_logger",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:file",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "demo_resonant_sound_pack",
    srcs = ["demo_resonant_sound_pack.cc"],
    hdrs = ["demo_resonant_sound_pack.h"],
    deps = [
        ":sound_pack",
        "//project_guideline/audio/assets:midleft_embed",
        "//project_guideline/audio/assets:midright_embed",
        "//project_guideline/audio/assets:obstacle_embed",
        "//project_guideline/audio/assets:sharpleft_embed",
        "//project_guideline/audio/assets:sharpright_embed",
        "//project_guideline/audio/assets:slightleft_embed",
        "//project_guideline/audio/assets:slightright_embed",
        "//project_guideline/audio/assets:steering_off_course_impulse_tick_embed",
        "//project_guideline/audio/assets:steering_on_course_impulse_tick_embed",
        "//project_guideline/audio/assets:straight_embed",
        "//project_guideline/audio/assets:warning_pitch_shift_2x_embed",
        "//project_guideline/environment:control_signal",
        "//project_guideline/util:status",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "legacy_sound_pack",
    srcs = ["legacy_sound_pack.cc"],
    hdrs = ["legacy_sound_pack.h"],
    deps = [
        ":sound_pack",
        ":sound_player",
        "//project_guideline/audio/assets:legacy_steering_20211014_embed",
        "//project_guideline/audio/assets:legacy_steering_embed",
        "//project_guideline/audio/assets:legacy_v4_1_steering_embed",
        "//project_guideline/audio/assets:legacy_v4_1_stop_embed",
        "//project_guideline/audio/assets:legacy_v4_1_warning_embed",
        "//project_guideline/audio/assets:legacy_v4_2_steering_embed",
        "//project_guideline/audio/assets:legacy_v4_2_stop_embed",
        "//project_guideline/audio/assets:legacy_v4_2_warning_embed",
        "//project_guideline/audio/assets:legacy_warning_embed",
        "//project_guideline/audio/assets:obstacle_embed",
        "//project_guideline/audio/assets:warning_pitch_shift_2x_embed",
        "//project_guideline/environment:control_signal",
        "//project_guideline/logging:guideline_logger",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/util:embedded_file_toc",
        "//project_guideline/util:lerp",
        "//project_guideline/util:status",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "resampler",
    srcs = ["resampler.cc"],
    hdrs = ["resampler.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
    ],
)

cc_test(
    name = "resampler_test",
    srcs = ["resampler_test.cc"],
    data = [
        "//project_guideline/audio/testdata:song.ogg",
    ],
    deps = [
        ":audio_buffer",
        ":resampler",
        ":vorbis_stream_decoder",
        ":vorbis_stream_encoder",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:file",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "sink_audio_output_stream",
    srcs = ["sink_audio_output_stream.cc"],
    hdrs = ["sink_audio_output_stream.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":audio_output_stream",
        "//project_guideline/util:clock",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/time",
    ],
)

cc_test(
    name = "sink_audio_output_stream_test",
    srcs = ["sink_audio_output_stream_test.cc"],
    deps = [
        ":sink_audio_output_stream",
        "//project_guideline/testing:simulated_clock",
        "//project_guideline/testing:status_matchers",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "sound_pack",
    srcs = ["sound_pack.cc"],
    hdrs = ["sound_pack.h"],
    deps = [
        ":sound_player",
        "//project_guideline/environment:control_signal",
        "//project_guideline/logging:guideline_logger",
        "//project_guideline/util:embedded_file_toc",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "sound_player",
    srcs = ["sound_player.cc"],
    hdrs = ["sound_player.h"],
    deps = [
        ":audio_stream_renderer",
        "//project_guideline/util:embedded_file_toc",
        "//project_guideline/util:status",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@resonance_audio",
    ],
)

cc_library(
    name = "vorbis_stream_decoder",
    srcs = ["vorbis_stream_decoder.cc"],
    hdrs = ["vorbis_stream_decoder.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":audio_buffer",
        "//project_guideline/util:status",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@libvorbis",
    ],
)

cc_test(
    name = "vorbis_stream_decoder_test",
    srcs = ["vorbis_stream_decoder_test.cc"],
    data = [
        "//project_guideline/audio/testdata:song.ogg",
    ],
    deps = [
        ":audio_buffer",
        ":vorbis_stream_decoder",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:file",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "vorbis_stream_encoder",
    srcs = ["vorbis_stream_encoder.cc"],
    hdrs = ["vorbis_stream_encoder.h"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":audio_buffer",
        ":vorbis_stream_decoder",
        "//project_guideline/util:status",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@libogg",
        "@libvorbis",
    ],
)

cc_test(
    name = "vorbis_stream_encoder_test",
    srcs = ["vorbis_stream_encoder_test.cc"],
    data = [
        "//project_guideline/audio/testdata:song.ogg",
    ],
    deps = [
        ":audio_buffer",
        ":vorbis_stream_decoder",
        ":vorbis_stream_encoder",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:file",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest_main",
    ],
)
