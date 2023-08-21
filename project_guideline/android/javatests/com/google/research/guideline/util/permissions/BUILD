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

load("@build_bazel_rules_android//android:rules.bzl", "android_local_test")
load("//project_guideline/android/javatests/com/google/research/guideline:build_defs.bzl", "GUIDELINE_DEFAULT_MANIFEST_VALUES")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = [
        "//:internal",
    ],
)

android_local_test(
    name = "RequiredPermissionsHelperTest",
    size = "small",
    srcs = ["RequiredPermissionsHelperTest.java"],
    manifest_values = GUIDELINE_DEFAULT_MANIFEST_VALUES,
    deps = [
        "//:hilt-android-testing",
        "//project_guideline/android/java/com/google/research/guideline/testing:runtime_permissions",
        "//project_guideline/android/java/com/google/research/guideline/testing:shell_activity",
        "//project_guideline/android/java/com/google/research/guideline/util/permissions",
        "//third_party/hilt:android_entry_point",
        "//third_party/hilt:qualifiers",
        "@maven//:androidx_appcompat_appcompat",
        "@maven//:androidx_fragment_fragment",
        "@maven//:androidx_test_core",
        "@maven//:androidx_test_espresso_espresso_core",
        "@maven//:androidx_test_espresso_espresso_intents",
        "@maven//:androidx_test_ext_junit",
        "@maven//:com_google_truth_truth",
        "@maven//:junit_junit",
        "@maven//:org_hamcrest_hamcrest_library",
        "@maven//:org_robolectric_annotations",
        "@maven//:org_robolectric_robolectric",
        "@maven//:org_robolectric_shadows_framework",
        "@robolectric//bazel:android-all",
    ],
)
