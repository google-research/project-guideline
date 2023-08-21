// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Plugin for running Guideline Engine from within Unreal as a shared library.
// This is the standalone public header file for the plugin entry-point.
// STL libraries (such as std::string and std::vector) are intentionally avoided
// since Unreal Engine compiles against libc++ instead of libstdc++ which
// results in interface incompatibilities.

#ifndef PROJECT_GUIDELINE_UNREAL_UNREAL_PLUGIN_H_
#define PROJECT_GUIDELINE_UNREAL_UNREAL_PLUGIN_H_

#include <cstddef>
#include <cstdint>

namespace guideline::unreal {

// A 2D vector.
struct GuidelineVector2 {
  float x = 0.;
  float y = 0.;
};

// A list of 2D points.
struct GuidelinePoints {
  GuidelineVector2* points = nullptr;
  size_t num_points = 0;
};

// Control signal data.
struct UnrealControlSignal {
  float lateral_movement_meters = 0.;
  float rotation_movement_degrees = 0.;
  GuidelineVector2 turn_point;
  float turn_angle_degrees = 0.;
  float turn_point_distance_meters = 0.;
  bool stop = true;
};

// Options to configure the Guideline engine.
struct UnrealGuidelineOptions {
  size_t audio_frames_per_buffer = -1;
  const char* file_logger_output_dir = "";
  bool log_intermediate = false;
  bool enable_conservative_mode = false;
  float conservative_mode_lane_width_meters = 1.6f;
  const char* config_dir = "";
};

// Indicates the position and direction of the runner.
struct GuidelineRunnerPose {
  GuidelineVector2 position;
  GuidelineVector2 direction;
};

// Listener that is notified of control signal updates.
class UnrealControlSignalListener {
 public:
  virtual ~UnrealControlSignalListener() {}
  virtual void OnControlSignal(const UnrealControlSignal& control_signal) = 0;
};

// Listener that is notified of log messages.
class UnrealDebugLogListener {
 public:
  virtual ~UnrealDebugLogListener() {}
  virtual void OnDebugLog(const char* message) = 0;
};

// The main entry point from Unreal Engine plugin.
class UnrealPlugin {
 public:
  virtual ~UnrealPlugin() = default;

  virtual void Start() = 0;
  virtual void Stop() = 0;

  virtual void SetControlSignalListener(
      UnrealControlSignalListener* listener) = 0;

  virtual void SetDebugLogListener(UnrealDebugLogListener* listener) = 0;

  virtual void OnTracking(bool tracking) = 0;

  // Invoked when the Unreal camera pose changes.
  // The translation [tx, ty, tz] is from Unreal GetCameraLocation(), and
  // the [qx, qy, qz, qw] is from GetCameraRotation().Quaternion().
  virtual void OnCameraPose(uint64_t timestamp_us, float tx, float ty, float tz,
                            float qx, float qy, float qz, float qw) = 0;

  virtual void OnImage(uint64_t timestamp_us, uint16_t width, uint16_t height,
                       void* data) = 0;

  virtual int32_t OnGenerateAudio(int16_t* out_buffer, int32_t num_samples) = 0;

  virtual GuidelineRunnerPose GetRunnerPose() = 0;

  // Gets the current guideline points, filling the given GuidelinePoints
  // struct. The memory must be released by a call to ReleaseGuidelinePoints.
  virtual void GetGuidelinePoints(GuidelinePoints* out) = 0;

  // Releases the allocated memory for the GuidelinePoints previously acquired
  // through GetGuidelinePoints.
  virtual void ReleaseGuidelinePoints(GuidelinePoints* points) = 0;

  virtual UnrealControlSignal GetLastControlSignal() = 0;

  virtual void GetCameraImageDimensions(float& width, float& height) = 0;

  virtual void GetProjectionMatrix(float near, float far,
                                   float* out_col_major_4x4) = 0;
};

UnrealPlugin* CreateUnrealPlugin(const UnrealGuidelineOptions& options);

}  // namespace guideline::unreal

#endif  // PROJECT_GUIDELINE_UNREAL_UNREAL_PLUGIN_H_
