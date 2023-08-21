load("@rules_foreign_cc//tools/build_defs:cmake.bzl", "cmake_external")

filegroup(
  name = "all_srcs",
  srcs = glob(["**"]),
)

# libEGL tries to load libGLESv2.so from the same directory, so both must be specified in the
# same cc_library otherwise bazel puts them in different directories.
cmake_external(
    name = "swiftshader",
    cache_entries = {
      "SWIFTSHADER_BUILD_GLES_CM": "FALSE",
      "SWIFTSHADER_BUILD_VULKAN": "FALSE",
      "SWIFTSHADER_BUILD_PVR": "FALSE",
      "SWIFTSHADER_BUILD_TESTS": "FALSE",
    },
    make_commands = [
        "cmake --build . --parallel=16",
        "mkdir -p $BUILD_TMPDIR/$INSTALL_PREFIX/lib",
        "cp $BUILD_TMPDIR/*.so $BUILD_TMPDIR/$INSTALL_PREFIX/lib",
    ],
    lib_source = ":all_srcs",
    shared_libraries = ["libEGL.so", "libGLESv2.so"],
    visibility = ["//visibility:public"],
)

# Convenience for targets that depend on EGL or GLESv2 individually.
cc_library(
    name = "EGL",
    visibility = ["//visibility:public"],
    deps = [":swiftshader"],
)

cc_library(
    name = "GLESv2",
    visibility = ["//visibility:public"],
    deps = [":swiftshader"],
)
