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
load("//third_party/bazel_rules/rules_cc/cc:cc_binary.bzl", "cc_binary")
load("//third_party/bazel_rules/rules_cc/cc:cc_test.bzl", "cc_test")

package(default_applicable_licenses = ["//:license"])

cc_binary(
    name = "filewrapper",
    srcs = ["filewrapper.cc"],
    deps = [
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

gl_cc_embed_data(
    name = "test_embed_single",
    srcs = ["testdata/test1.bin"],
)

gl_cc_embed_data(
    name = "test_embed_multiple",
    srcs = [
        "testdata/test1.bin",
        "testdata/test2.bin",
        "testdata/test3.bin",
    ],
    namespace = "testns",
)

cc_test(
    name = "filewrapper_test",
    srcs = ["filewrapper_test.cc"],
    data = [
        "testdata/test1.bin",
        "testdata/test2.bin",
        "testdata/test3.bin",
    ],
    deps = [
        ":test_embed_multiple",
        ":test_embed_single",
        "//project_guideline/testing:status_matchers",
        "//project_guideline/util:embedded_file_toc",
        "//project_guideline/util:file",
        "@com_google_absl//absl/flags:flag",
        "@com_google_googletest//:gtest_main",
    ],
)
