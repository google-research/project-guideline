# Project Guideline Android Application

This directory contains a reference Android application implementation for
Project Guideline. See the [main documentation][root-readme] for build
and usage instructions.

[root-readme]: https://github.com/google-research/project-guideline/#readme

## ARCore Tracking

The Android app uses the [ARCore SDK][arcore-sdk] to provide 6DoF
(6 degrees of freedom) tracking of the phone camera along with the camera
images. The implementation is available in the [arcore][arcore-dir] directory.

[arcore-sdk]: https://developers.google.com/ar
[arcore-dir]: https://github.com/google-research/project-guideline/tree/main/project_guideline/android/arcore

## AAudio

The [AAudio API][aaudio-api] is used to play low-latency sound through the
device. The *AudioOutputStream* implementation that interfaces with the
Guideline audio system is available in the [audio][audio-dir] directory.

[aaudio-api]: https://developer.android.com/ndk/guides/audio/aaudio/aaudio
[audio-dir]: https://github.com/google-research/project-guideline/tree/main/project_guideline/android/audio

## Configuration

There are several options and parameters that can be configured, for example to
change the sounds or navigation thresholds. These are documented in
[guideline_engine_config.proto][config-proto], and can be changed during the
system creation in [NativeEngineFragment.java][native-engine-fragment].

[config-proto]: https://github.com/google-research/project-guideline/blob/main/project_guideline/proto/guideline_engine_config.proto
[native-engine-fragment]: https://github.com/google-research/project-guideline/blob/main/project_guideline/android/java/com/google/research/guideline/engine/NativeEngineFragment.java
