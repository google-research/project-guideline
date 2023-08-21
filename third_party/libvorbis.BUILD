licenses(["notice"])

exports_files(["LICENSE"])

cc_library(
    name = "libvorbis",
    srcs = [
        "lib/analysis.c",
        "lib/bitrate.c",
        "lib/block.c",
        "lib/codebook.c",
        "lib/envelope.c",
        "lib/floor0.c",
        "lib/floor1.c",
        "lib/info.c",
        "lib/lookup.c",
        "lib/lpc.c",
        "lib/lsp.c",
        "lib/mapping0.c",
        "lib/mdct.c",
        "lib/psy.c",
        "lib/registry.c",
        "lib/res0.c",
        "lib/sharedbook.c",
        "lib/smallft.c",
        "lib/synthesis.c",
        "lib/vorbisenc.c",
        "lib/vorbisfile.c",
        "lib/window.c",
    ],
    hdrs = glob([
        "*.h",
        "include/vorbis/*.h",
        "lib/**/*.h",
    ]),
    copts = [
        "-DUSE_MEMORY_H",
        "-D_REENTRANT",
        "-Iexternal/libvorbis/lib",
        "-w ",
    ],
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = ["@libogg"],
)