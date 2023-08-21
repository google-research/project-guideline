licenses(["notice"])

exports_files(["LICENSE"])

cc_library(
    name = "resonance_audio",
    hdrs = glob(["resonance_audio/api/*.h"]),
    includes = ["resonance_audio/api"],
    visibility = ["//visibility:public"],
    deps = [":resonance_audio_impl"],
)

cc_library(
    name = "resonance_audio_impl",
    srcs = glob(
        [
            "platforms/common/*.cc",
            "resonance_audio/**/*.cc",
            "third_party/SADIE_hrtf_database/generated/*.cc",
        ],
        exclude = [
            "**/*test.cc",
            "resonance_audio/utils/test_util.cc",
            "resonance_audio/geometrical_acoustics/*.cc",
        ],
    ),
    hdrs = glob(
        [
            "platforms/common/*.h",
            "resonance_audio/**/*.h",
            "third_party/SADIE_hrtf_database/generated/*.h",
        ],
        exclude = [
            "resonance_audio/utils/test_util.h",
            "resonance_audio/geometrical_acoustics/*.h",
        ],
    ),
    copts = [
        "-Iexternal/resonance_audio/resonance_audio",
        "-DGOOGLE_PROTOBUF_NO_RTTI=1",
        "-fno-rtti",
    ],
    deps = [
        "@eigen_archive//:eigen3",
        "@libogg",
        "@libvorbis",
        "@pffft",
    ],
)
