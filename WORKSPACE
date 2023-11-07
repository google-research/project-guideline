workspace(name = "project_guideline")

# Loads dependencies required to build Project Guideline. Many of these are copied from the
# MediaPipe WORKSPACE file since there is no way to transitively include another WORKSPACE file
# and MediaPipe does not provide Bazel macros to load dependencies (like TensorFlow does with
# tf_workspace()).

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "mediapipe",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@//third_party:mediapipe_patch.diff",
    ],
    sha256 = "2d01c0b98ca6bef2ea048b37f0a99da19b91dff503ef6043bce1e61a2c3253e9",
    strip_prefix = "mediapipe-0.9.3.0",
    urls = ["https://github.com/google/mediapipe/archive/refs/tags/v0.9.3.0.zip"],
)

http_archive(
    name = "bazel_skylib",
    sha256 = "74d544d96f4a5bb630d465ca8bbcfe231e3594e5aae57e1edbf17a6eb3ca2506",
    urls = [
        "https://storage.googleapis.com/mirror.tensorflow.org/github.com/bazelbuild/bazel-skylib/releases/download/1.3.0/bazel-skylib-1.3.0.tar.gz",
        "https://github.com/bazelbuild/bazel-skylib/releases/download/1.3.0/bazel-skylib-1.3.0.tar.gz",
    ],
)

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

bazel_skylib_workspace()

load("@bazel_skylib//lib:versions.bzl", "versions")

versions.check(minimum_bazel_version = "3.7.2")

http_archive(
    name = "zlib",
    build_file = "@mediapipe//third_party:zlib.BUILD",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@mediapipe//third_party:zlib.diff",
    ],
    sha256 = "c3e5e9fdd5004dcb542feda5ee4f0ff0744628baf8ed2dd5d66f8ca1197cb1a1",
    strip_prefix = "zlib-1.2.11",
    urls = [
        "http://mirror.bazel.build/zlib.net/fossils/zlib-1.2.11.tar.gz",
        "http://zlib.net/fossils/zlib-1.2.11.tar.gz",
    ],
)

# iOS basic build deps.
http_archive(
    name = "build_bazel_rules_apple",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@mediapipe//third_party:build_bazel_rules_apple_bypass_test_runner_check.diff",
    ],
    sha256 = "3e2c7ae0ddd181c4053b6491dad1d01ae29011bc322ca87eea45957c76d3a0c3",
    url = "https://github.com/bazelbuild/rules_apple/releases/download/2.1.0/rules_apple.2.1.0.tar.gz",
)

load(
    "@build_bazel_rules_apple//apple:repositories.bzl",
    "apple_rules_dependencies",
)

apple_rules_dependencies()

load(
    "@build_bazel_apple_support//lib:repositories.bzl",
    "apple_support_dependencies",
)

apple_support_dependencies()

http_archive(
    name = "rules_license",
    sha256 = "6157e1e68378532d0241ecd15d3c45f6e5cfd98fc10846045509fb2a7cc9e381",
    urls = [
        "https://github.com/bazelbuild/rules_license/releases/download/0.0.4/rules_license-0.0.4.tar.gz",
        "https://mirror.bazel.build/github.com/bazelbuild/rules_license/releases/download/0.0.4/rules_license-0.0.4.tar.gz",
    ],
)

http_archive(
    name = "rules_jvm_external",
    sha256 = "cd1a77b7b02e8e008439ca76fd34f5b07aecb8c752961f9640dea15e9e5ba1ca",
    strip_prefix = "rules_jvm_external-4.2",
    url = "https://github.com/bazelbuild/rules_jvm_external/archive/4.2.zip",
)

load("@rules_jvm_external//:repositories.bzl", "rules_jvm_external_deps")

rules_jvm_external_deps()

load("@rules_jvm_external//:setup.bzl", "rules_jvm_external_setup")

rules_jvm_external_setup()

load("@rules_jvm_external//:defs.bzl", "maven_install")

http_archive(
    name = "dagger",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@//third_party:dagger_patch.diff",
    ],
    sha256 = "8f55f8b081a72835fed96be66dbfccb95ee2ceff092872397e5e639cccec2114",
    strip_prefix = "dagger-dagger-2.45",
    urls = ["https://github.com/google/dagger/archive/dagger-2.45.zip"],
)

load(
    "@dagger//:workspace_defs.bzl",
    "DAGGER_ANDROID_ARTIFACTS",
    "DAGGER_ANDROID_REPOSITORIES",
)

maven_install(
    artifacts = [
        "androidx.core:core:1.6.0",
        "androidx.test:core:1.4.0",
        "androidx.test.ext:junit:1.1.4",
        "androidx.test.espresso:espresso-core:3.5.0",
        "androidx.test.espresso:espresso-intents:3.5.0",
        "org.robolectric:robolectric:4.9.2",
        "org.robolectric:shadows-framework:4.9.2",
        "com.google.ar:core:1.36.0",
        "com.google.code.findbugs:jsr305:3.0.2",
        "com.google.code.gson:gson:2.8.9",
        "com.google.guava:guava:31.1-android",
        "com.google.truth:truth:1.1.3",
        "javax.annotation:javax.annotation-api:1.3.2",
        "androidx.annotation:annotation:1.5.0",
        "com.google.dagger:hilt-android:2.46",
        "com.google.dagger:hilt-android-testing:2.46",
        "com.google.dagger:hilt-android-compiler:2.46",
        "com.google.dagger:hilt-core:2.46",
    ] + DAGGER_ANDROID_ARTIFACTS,
    fetch_sources = True,
    generate_compat_repositories = True,
    repositories = [
        "https://maven.google.com",
        "https://repo1.maven.org/maven2",
    ] + DAGGER_ANDROID_REPOSITORIES,
)

http_archive(
    name = "com_google_absl",
    sha256 = "3ea49a7d97421b88a8c48a0de16c16048e17725c7ec0f1d3ea2683a2a75adc21",
    strip_prefix = "abseil-cpp-20230125.0",
    urls = [
        "https://github.com/abseil/abseil-cpp/archive/refs/tags/20230125.0.tar.gz",
    ],
)

http_archive(
    name = "rules_cc",
    sha256 = "691a29db9c336349e48e04c5c2f4873f2890af5cbfa6e51f4de87fefe6169294",
    strip_prefix = "rules_cc-2f8c04c04462ab83c545ab14c0da68c3b4c96191",
    urls = ["https://github.com/bazelbuild/rules_cc/archive/2f8c04c04462ab83c545ab14c0da68c3b4c96191.zip"],
)

http_archive(
    name = "rules_foreign_cc",
    sha256 = "c2cdcf55ffaf49366725639e45dedd449b8c3fe22b54e31625eb80ce3a240f1e",
    strip_prefix = "rules_foreign_cc-0.1.0",
    url = "https://github.com/bazelbuild/rules_foreign_cc/archive/0.1.0.zip",
)

load("@rules_foreign_cc//:workspace_definitions.bzl", "rules_foreign_cc_dependencies")

rules_foreign_cc_dependencies()

http_archive(
    name = "build_bazel_rules_android",
    sha256 = "cd06d15dd8bb59926e4d65f9003bfc20f9da4b2519985c27e190cddc8b7a7806",
    strip_prefix = "rules_android-0.1.1",
    urls = ["https://github.com/bazelbuild/rules_android/archive/v0.1.1.zip"],
)

http_archive(
    name = "org_tensorflow",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@mediapipe//third_party:org_tensorflow_compatibility_fixes.diff",
        "@mediapipe//third_party:org_tensorflow_custom_ops.diff",
    ],
    sha256 = "ba98de6ea5f720071246691a1536ecd5e1b1763033e8c82a1e721a06d3dfd4c1",
    strip_prefix = "tensorflow-d712c0c9e24519cc8cd3720279666720d1000eee",
    urls = [
        "https://github.com/tensorflow/tensorflow/archive/d712c0c9e24519cc8cd3720279666720d1000eee.tar.gz",
    ],
)

load("@org_tensorflow//tensorflow:workspace3.bzl", "tf_workspace3")

tf_workspace3()

load("@org_tensorflow//tensorflow:workspace2.bzl", "tf_workspace2")

tf_workspace2()

# Edge TPU
http_archive(
    name = "libedgetpu",
    sha256 = "14d5527a943a25bc648c28a9961f954f70ba4d79c0a9ca5ae226e1831d72fe80",
    strip_prefix = "libedgetpu-3164995622300286ef2bb14d7fdc2792dae045b7",
    urls = [
        "https://github.com/google-coral/libedgetpu/archive/3164995622300286ef2bb14d7fdc2792dae045b7.tar.gz",
    ],
)

load("@libedgetpu//:workspace.bzl", "libedgetpu_dependencies")

libedgetpu_dependencies()

load("@coral_crosstool//:configure.bzl", "cc_crosstool")

cc_crosstool(name = "crosstool")

http_archive(
    name = "com_google_protobuf_javalite",
    sha256 = "87407cd28e7a9c95d9f61a098a53cf031109d451a7763e7dd1253abf8b4df422",
    strip_prefix = "protobuf-3.19.1",
    urls = ["https://github.com/protocolbuffers/protobuf/archive/v3.19.1.tar.gz"],
)

http_archive(
    name = "com_google_googletest",
    sha256 = "de682ea824bfffba05b4e33b67431c247397d6175962534305136aa06f92e049",
    strip_prefix = "googletest-4ec4cd23f486bf70efcc5d2caa40f24368f752e3",
    urls = ["https://github.com/google/googletest/archive/4ec4cd23f486bf70efcc5d2caa40f24368f752e3.zip"],
)

http_archive(
    name = "com_google_benchmark",
    build_file = "@//third_party:benchmark.BUILD",
    sha256 = "6132883bc8c9b0df5375b16ab520fac1a85dc9e4cf5be59480448ece74b278d4",
    strip_prefix = "benchmark-1.6.1",
    urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.6.1.tar.gz"],
)

# gflags needed by glog
http_archive(
    name = "com_github_gflags_gflags",
    sha256 = "19713a36c9f32b33df59d1c79b4958434cb005b5b47dc5400a7a4b078111d9b5",
    strip_prefix = "gflags-2.2.2",
    url = "https://github.com/gflags/gflags/archive/v2.2.2.zip",
)

http_archive(
    name = "com_github_glog_glog",
    sha256 = "58c9b3b6aaa4dd8b836c0fd8f65d0f941441fb95e27212c5eeb9979cfd3592ab",
    strip_prefix = "glog-0a2e5931bd5ff22fd3bf8999eb8ce776f159cda6",
    urls = [
        "https://github.com/google/glog/archive/0a2e5931bd5ff22fd3bf8999eb8ce776f159cda6.zip",
    ],
)

http_archive(
    name = "com_github_glog_glog_no_gflags",
    build_file = "@mediapipe//third_party:glog_no_gflags.BUILD",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@mediapipe//third_party:com_github_glog_glog_9779e5ea6ef59562b030248947f787d1256132ae.diff",
    ],
    sha256 = "58c9b3b6aaa4dd8b836c0fd8f65d0f941441fb95e27212c5eeb9979cfd3592ab",
    strip_prefix = "glog-0a2e5931bd5ff22fd3bf8999eb8ce776f159cda6",
    urls = [
        "https://github.com/google/glog/archive/0a2e5931bd5ff22fd3bf8999eb8ce776f159cda6.zip",
    ],
)

all_content = """filegroup(name = "all", srcs = glob(["**"]), visibility = ["//visibility:public"])"""

http_archive(
    name = "opencv",
    build_file_content = all_content,
    sha256 = "1ed6f5b02a7baf14daca04817566e7c98ec668cec381e0edf534fa49f10f58a2",
    strip_prefix = "opencv-3.4.10",
    urls = ["https://github.com/opencv/opencv/archive/3.4.10.tar.gz"],
)

new_local_repository(
    name = "linux_opencv",
    build_file = "@//third_party:opencv_linux.BUILD",
    path = "third_party", # unused
)

http_archive(
    name = "android_opencv",
    build_file = "@mediapipe//third_party:opencv_android.BUILD",
    sha256 = "cdb0e190c3734edd4052a3535d9e4310af912a9f70a421b1621711942a1028d5",
    strip_prefix = "OpenCV-android-sdk",
    type = "zip",
    url = "https://github.com/opencv/opencv/releases/download/3.4.3/opencv-3.4.3-android-sdk.zip",
)

http_archive(
    name = "stblib",
    build_file = "@mediapipe//third_party:stblib.BUILD",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@mediapipe//third_party:stb_image_impl.diff",
    ],
    sha256 = "13a99ad430e930907f5611325ec384168a958bf7610e63e60e2fd8e7b7379610",
    strip_prefix = "stb-b42009b3b9d4ca35bc703f5310eedc74f584be58",
    urls = ["https://github.com/nothings/stb/archive/b42009b3b9d4ca35bc703f5310eedc74f584be58.tar.gz"],
)

load("@mediapipe//third_party:external_files.bzl", "external_files")

external_files()

http_archive(
    name = "pffft",
    build_file = "@mediapipe//third_party:pffft.BUILD",
    sha256 = "20f48fbbd5737d9a7db419f216c8d01ec8fdead48442c04774425a4c6b8b3949",
    strip_prefix = "jpommier-pffft-7c3b5a7dc510",
    urls = ["https://bitbucket.org/jpommier/pffft/get/7c3b5a7dc510.zip"],
)

http_archive(
    name = "libogg",
    build_file = "@//third_party:libogg.BUILD",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@//third_party:libogg_patch.diff",
    ],
    sha256 = "0eb4b4b9420a0f51db142ba3f9c64b333f826532dc0f48c6410ae51f4799b664",
    strip_prefix = "libogg-1.3.5",
    urls = ["https://downloads.xiph.org/releases/ogg/libogg-1.3.5.tar.gz"],
)

http_archive(
    name = "libvorbis",
    build_file = "@//third_party:libvorbis.BUILD",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@//third_party:libvorbis_patch.diff",
    ],
    sha256 = "0e982409a9c3fc82ee06e08205b1355e5c6aa4c36bca58146ef399621b0ce5ab",
    strip_prefix = "libvorbis-1.3.7",
    urls = ["https://ftp.osuosl.org/pub/xiph/releases/vorbis/libvorbis-1.3.7.tar.gz"],
)

http_archive(
    name = "resonance_audio",
    build_file = "@//third_party:resonance_audio.BUILD",
    sha256 = "f7f3fbb53dbcd4f14812a7e4c2825c37423f45487a4710a34ff3f922a5efec77",
    strip_prefix = "resonance-audio-4556a46afd4ffae092aa281bfd072eb0279d3a29",
    urls = ["https://github.com/resonance-audio/resonance-audio/archive/4556a46afd4ffae092aa281bfd072eb0279d3a29.tar.gz"],
)

http_archive(
    name = "arcore_android_sdk",
    build_file = "@//third_party:arcore_android_sdk.BUILD",
    sha256 = "4451b8965741a68da4a9bdda24a0cb87f8d181009086bd7a221350afa26589e1",
    strip_prefix = "arcore-android-sdk-1.36.0",
    urls = ["https://github.com/google-ar/arcore-android-sdk/archive/refs/tags/v1.36.0.tar.gz"],
)

http_archive(
    name = "arcore_android_aar",
    build_file = "@//third_party:arcore_android_aar.BUILD",
    sha256 = "1c99dbb7c14a67a3e157e45d04744cac7484d1f1afdace56ce3f47c122935f9f",
    type = "zip",
    urls = ["https://maven.google.com/com/google/ar/core/1.36.0/core-1.36.0.aar"],
)

http_archive(
    name = "libyuv",
    build_file = "@mediapipe//third_party:libyuv.BUILD",
    # SHA256 is not consistent for this URL.
    urls = ["https://chromium.googlesource.com/libyuv/libyuv/+archive/2525698acba9bf9b701ba6b4d9584291a1f62257.tar.gz"],
)

http_archive(
    name = "libpng",
    urls = ["https://fuchsia.googlesource.com/third_party/libpng/+archive/a40189cf881e9f0db80511c382292a5604c3c3d1.tar.gz"],
    build_file = "//third_party:libpng.BUILD",
)

http_archive(
    name = "robolectric",
    sha256 = "7e007fcfdca7b7228cb4de72707e8b317026ea95000f963e91d5ae365be52d0d",
    urls = ["https://github.com/robolectric/robolectric-bazel/archive/4.9.2.tar.gz"],
    strip_prefix = "robolectric-bazel-4.9.2",
)

load("@robolectric//bazel:robolectric.bzl", "robolectric_repositories")
robolectric_repositories()

http_archive(
    name = "swiftshader",
    build_file = "@//third_party:swiftshader.BUILD",
    sha256 = "4fc289e15ae343b65e6ed6be631d1e807c2b2866dbfb1f45e8d185871e02ceba",
    strip_prefix = "swiftshader-9677c6d282788b0e2b7e2a02d539b5fa86279ee1",
    urls = ["https://github.com/google/swiftshader/archive/9677c6d282788b0e2b7e2a02d539b5fa86279ee1.tar.gz"],
)
