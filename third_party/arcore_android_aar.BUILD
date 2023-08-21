cc_import(
    name = "arcore_sdk_c",
    shared_library =
        select({
            "@mediapipe//mediapipe:android_arm64": "jni/arm64-v8a/libarcore_sdk_c.so",
            "@mediapipe//mediapipe:android_armeabi": "jni/armeabi-v7a/libarcore_sdk_c.so",
            "@mediapipe//mediapipe:android_arm": "jni/armeabi-v7a/libarcore_sdk_c.so",
            "@mediapipe//mediapipe:android_x86": "jni/x86/libarcore_sdk_c.so",
            "@mediapipe//mediapipe:android_x86_64": "jni/x86_64/libarcore_sdk_c.so",
            "//conditions:default": "jni/arm64-v8a/libarcore_sdk_c.so",
        }),
    visibility = ["//visibility:public"],
)
