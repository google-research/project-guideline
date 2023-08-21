genrule(
    name = "pnglibconf",
    srcs = ["scripts/pnglibconf.h.prebuilt"],
    outs = ["pnglibconf.h"],
    cmd = "cp $< $@",
)

cc_library(
    name = "libpng",
    hdrs = ["png.h"],
    includes = ["."],
    srcs = [
        "png.c",
        "pngerror.c",
        "pngget.c",
        "pngmem.c",
        "pngpread.c",
        "pngread.c",
        "pngrio.c",
        "pngrtran.c",
        "pngrutil.c",
        "pngset.c",
        "pngtrans.c",
        "pngwio.c",
        "pngwrite.c",
        "pngwtran.c",
        "pngwutil.c",
    ] + [
        ":pnglibconf",
        "pngconf.h",
        "pngdebug.h",
        "pnginfo.h",
        "pngpriv.h",
        "pngstruct.h",
    ],
    deps = ["@bazel_tools//third_party/zlib"],
    visibility = ["//visibility:public"],
)
