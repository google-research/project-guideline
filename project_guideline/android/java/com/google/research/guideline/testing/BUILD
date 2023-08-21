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

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = [
        "//:internal",
    ],
)

android_library(
    name = "runtime_permissions",
    testonly = 1,
    srcs = ["RuntimePermissions.java"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "@maven//:androidx_test_espresso_espresso_intents",
        "@maven//:org_hamcrest_hamcrest_core",
        "@maven//:org_robolectric_shadows_framework",
    ],
)

android_library(
    name = "shell_activity",
    testonly = 1,
    srcs = ["ShellActivity.java"],
    exports_manifest = 1,
    manifest = "ShellActivityManifest.xml",
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//third_party/hilt:android_entry_point",
        "@maven//:androidx_appcompat_appcompat",
    ],
)
