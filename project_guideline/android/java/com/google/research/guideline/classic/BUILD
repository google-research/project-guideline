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

load("@build_bazel_rules_android//android:rules.bzl", "android_library")
load("//project_guideline/bazel:build_defs.bzl", "android_only")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = [
        "//:internal",
    ],
)

android_library(
    name = "classic",
    srcs = glob(["*.java"]),
    assets = [
        "//third_party/project_guideline/android/mediapipe/graphs:mobile_graphs",
        "//third_party/project_guideline/vision/models:guideline_f32input_singlechannel.tflite",
    ],
    exports_manifest = 1,
    manifest = "AndroidManifest.xml",
    resource_files = glob(["res/**"]),
    deps = [
        "//:dagger-android",
        "//project_guideline/android/java/com/google/research/guideline/inject",
        "//project_guideline/android/java/com/google/research/guideline/util/ui",
        "//project_guideline/android/mediapipe/proto:classic_guidance_java_proto_lite",
        "//project_guideline/android/mediapipe/proto:line_detection_java_proto_lite",
        "//third_party/hilt:android_entry_point",
        "//third_party/hilt:install_in",
        "//third_party/hilt:scopes",
        "//third_party/java/android_libs/guava_jdk5:base",
        "//third_party/java/android_libs/guava_jdk5:concurrent",
        "//third_party/java/auto:auto_factory",
        "//third_party/java/auto:auto_value",
        "//third_party/protobuf:duration_java_proto_lite",
        "@maven//:androidx_annotation_annotation",
        "@maven//:androidx_appcompat_appcompat",
        "@maven//:androidx_camera_camera_core",
        "@maven//:androidx_fragment_fragment",
        "@maven//:androidx_lifecycle_lifecycle_common",
        "@maven//:com_google_code_findbugs_jsr305",
        "@maven//:javax_inject_javax_inject",
        "@mediapipe//mediapipe/java/com/google/mediapipe/components:android_camerax_helper",
        "@mediapipe//mediapipe/java/com/google/mediapipe/components:android_components",
        "@mediapipe//mediapipe/java/com/google/mediapipe/framework:android_framework",
        "@mediapipe//mediapipe/java/com/google/mediapipe/glutil",
    ] + android_only(":native_libs"),
)

android_library(
    name = "native_libs",
    exports = [
        "//third_party/project_guideline/android/mediapipe:mediapipe_jni_lib",
    ],
)
