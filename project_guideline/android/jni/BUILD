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
    name = "guideline_jni_lib",
    srcs = [":libguideline_native_jni.so"],
    target_compatible_with = [
        "@platforms//os:android",
    ],
    alwayslink = 1,
)

BASE_LINK_OPTS = [
    # Enable Android features
    "-landroid",
    "-laaudio",
    "-llog",
    # Enable graphics support.
    "-ljnigraphics",
    "-lGLESv3",
    "-lEGL",
    "-Wl,--no-undefined",  # Ensure there are no missing dependencies.
    "-funwind-tables",  # Preserves stacktraces.
    "-z defs",  # Report undefined symbols, making them a build error.
    "-Wl,--build-id",  # Add a section that uniquely identifies this build.
]

cc_binary(
    name = "libguideline_native_jni.so",
    linkopts = BASE_LINK_OPTS + [
        "-Wl,--undefined='JNI_OnLoad'",
        "-Wl,--undefined='JNI_OnUnload'",
        "-Wl,-soname,libguideline_native_jni.so",
        "-Wl,-version-script,$(location :jni.lds)",
    ],
    linkshared = 1,
    linkstatic = 1,
    deps = [
        ":guideline_jni_interface",
        ":jni.lds",
        "@arcore_android_aar//:arcore_sdk_c",
    ],
)

cc_library(
    name = "guideline_jni_interface",
    srcs = ["guideline_jni_interface.cc"],
    target_compatible_with = [
        "@platforms//os:android",
    ],
    deps = [
        ":guideline_engine_wrapper",
        ":jni_helpers",
    ],
)

cc_library(
    name = "guideline_engine_wrapper",
    srcs = ["guideline_engine_wrapper.cc"],
    hdrs = ["guideline_engine_wrapper.h"],
    target_compatible_with = [
        "@platforms//os:android",
    ],
    deps = [
        ":jni_helpers",
        "//project_guideline/android/arcore:arcore_data_source",
        "//project_guideline/android/arcore:arcore_motion_tracker",
        "//project_guideline/android/arcore:arcore_session",
        "//project_guideline/android/audio:android_audio_output_stream",
        "//project_guideline/audio:audio_output_stream",
        "//project_guideline/engine:guideline_engine",
        "//project_guideline/environment:control_signal",
        "//project_guideline/logging:guideline_logger",
        "//project_guideline/logging:noop_guideline_logger",
        "//project_guideline/proto:guideline_engine_config_cc_proto",
        "//project_guideline/util:status",
        "//project_guideline/visualization:camera_feed_renderer",
        "//project_guideline/visualization:environment_map_renderer",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@eigen_archive//:eigen3",
        "@org_tensorflow//tensorflow/lite/delegates/gpu:delegate",  # keep dep
    ],
)

cc_library(
    name = "jni_helpers",
    srcs = ["jni_helpers.cc"],
    hdrs = ["jni_helpers.h"],
    target_compatible_with = [
        "@platforms//os:android",
    ],
    deps = [
        "@com_google_absl//absl/log:check",
    ],
)
