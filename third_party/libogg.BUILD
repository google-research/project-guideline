licenses(["notice"])

exports_files(["LICENSE"])

cc_library(
    name = "libogg",
    srcs = [
        "src/bitwise.c",
        "src/crctable.h",
        "src/framing.c",
    ],
    hdrs = glob([
        "include/ogg/*.h",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = select({
        "@mediapipe//mediapipe:android": ["@androidndk//:cpufeatures"],
        "//conditions:default": [],
    }),
)
