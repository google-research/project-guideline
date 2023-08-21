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

#include "project_guideline/unreal/unreal_plugin.h"

#include <cmath>
#include <memory>
#include <optional>
#include <vector>

#include "gtest/gtest.h"
#include "absl/functional/bind_front.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "project_guideline/camera/camera_model.h"
#include "project_guideline/motion/motion_tracker.h"
#include "project_guideline/proto/guideline_log.pb.h"
#include "project_guideline/testing/predicates.h"
#include "project_guideline/unreal/unreal_plugin_impl.h"
#include "project_guideline/util/transformation.h"

namespace guideline::unreal {

namespace {

const size_t kNumChannels = 2;
const size_t kFramesPerBuffer = 8192;
const int kSampleRate = 48000;
const size_t kImageWidth = 64;
const size_t kImageHeight = 48;
const size_t kLineWidth = 4;

using util::Transformation;

class TestControlSignalListener : public UnrealControlSignalListener {
 public:
  void OnControlSignal(const UnrealControlSignal& control_signal) override {
    absl::MutexLock lock(&mutex_);
    control_signals_.push_back(control_signal);
  }

  std::vector<UnrealControlSignal> ControlSignals() {
    absl::MutexLock lock(&mutex_);
    return control_signals_;
  }

  void WaitForNumSignals(size_t num_signals, absl::Duration timeout) {
    auto value_condition = [num_signals, this]()
                               ABSL_SHARED_LOCKS_REQUIRED(mutex_) {
                                 return control_signals_.size() == num_signals;
                               };
    absl::MutexLock lock(&mutex_);
    mutex_.AwaitWithTimeout(absl::Condition(&value_condition), timeout);
  }

  absl::Mutex mutex_;
  std::vector<UnrealControlSignal> control_signals_ ABSL_GUARDED_BY(mutex_);
};

class LogEventListener {
 public:
  void OnLogEvent(const GuidelineLogEvent& event) {
    absl::MutexLock lock(&mutex_);
    if (event.has_line_detector_results()) {
      ++num_line_detector_results_;
    }
  }

  void WaitForNumLineDetectorResults(size_t num_results,
                                     absl::Duration timeout) {
    auto value_condition = [num_results,
                            this]() ABSL_SHARED_LOCKS_REQUIRED(mutex_) {
      return num_line_detector_results_ == num_results;
    };
    absl::MutexLock lock(&mutex_);
    mutex_.AwaitWithTimeout(absl::Condition(&value_condition), timeout);
  }

 private:
  absl::Mutex mutex_;
  int num_line_detector_results_ ABSL_GUARDED_BY(mutex_) = 0;
};

std::vector<uint8_t> CreateBGRAImage(float line_pos) {
  const size_t kPixelStride = 4;
  std::vector<uint8_t> image;
  image.resize(kImageWidth * kImageHeight * kPixelStride);

  size_t line_start_x = (kImageWidth * line_pos) - (kLineWidth / 2);
  size_t line_end_x = line_start_x + kLineWidth;

  uint8_t* ptr = image.data();
  for (size_t y = 0; y < kImageHeight; y++) {
    for (size_t x = 0; x < kImageWidth; x++) {
      // bgra
      if (x >= line_start_x && x < line_end_x) {
        ptr[0] = 0xF0;
        ptr[1] = 0x20;
        ptr[2] = 0xA0;
      } else {
        ptr[0] = 0;
        ptr[1] = 0;
        ptr[2] = 0;
      }
      ptr[3] = 0xFF;
      ptr += kPixelStride;
    }
  }

  return image;
}

void ReadAudio(UnrealPlugin& plugin, std::vector<int16_t>& buffer,
               absl::Duration at_least_duration) {
  double frames = absl::ToDoubleSeconds(at_least_duration) * kSampleRate;
  size_t num_buffers = std::ceil(frames / kFramesPerBuffer);

  buffer.resize(num_buffers * kFramesPerBuffer * kNumChannels);

  for (size_t i = 0; i < num_buffers; ++i) {
    plugin.OnGenerateAudio(&buffer[i * kFramesPerBuffer], kFramesPerBuffer);
  }
}

}  // namespace

TEST(UnrealPlugin, TestControlSignals) {
  UnrealGuidelineOptions options;
  options.audio_frames_per_buffer = kFramesPerBuffer;
  std::unique_ptr<UnrealPlugin> plugin =
      absl::WrapUnique(CreateUnrealPlugin(options));

  TestControlSignalListener listener;
  plugin->SetControlSignalListener(&listener);

  plugin->Start();
  plugin->OnTracking(true);

  uint64_t timestamp_us = 1000;
  std::vector<uint8_t> image = CreateBGRAImage(0.1);

  LogEventListener event_listener;
  auto& impl = static_cast<UnrealPluginImpl&>(*plugin);
  impl.SetLogEventCallback(
      absl::bind_front(&LogEventListener::OnLogEvent, &event_listener));

  const size_t kNumPoses = 20;
  for (int i = 0; i < kNumPoses; i++) {
    plugin->OnCameraPose(timestamp_us, 0., 0., 0., -0.000175, 0.235815,
                         -0.021034, 0.971570);
    plugin->OnImage(timestamp_us, kImageWidth, kImageHeight, image.data());
    timestamp_us += 100000;

    // The ML processing can be slow in tests. This ensures that we wait
    // for the segmentation results to be received by the system before
    // continuing.
    event_listener.WaitForNumLineDetectorResults(i + 1, absl::Seconds(5));
  }

  // We should get at least a few callbacks by now.
  listener.WaitForNumSignals(5, absl::Seconds(5));
  ASSERT_GE(listener.ControlSignals().size(), 5);

  std::optional<UnrealControlSignal> control_signal =
      listener.ControlSignals()[5];
  ASSERT_TRUE(control_signal.has_value());
  ASSERT_LT(control_signal->lateral_movement_meters, -0.15);

  GuidelinePoints points;
  plugin->GetGuidelinePoints(&points);
  ASSERT_GT(points.num_points, 0);
  for (int i = 0; i < points.num_points; i++) {
    EXPECT_NE(points.points[i].x, 0);
    EXPECT_NE(points.points[i].y, 0);
  }
  plugin->ReleaseGuidelinePoints(&points);

  plugin->Stop();
  plugin.reset();
}

TEST(UnrealPlugin, TestAudio) {
  UnrealGuidelineOptions options;
  options.audio_frames_per_buffer = kFramesPerBuffer;
  std::unique_ptr<UnrealPlugin> plugin =
      absl::WrapUnique(CreateUnrealPlugin(options));

  plugin->Start();

  std::vector<int16_t> buffer;
  ReadAudio(*plugin, buffer, absl::Seconds(1));

  float average = 0.;
  for (int16_t value : buffer) {
    average += value;
  }
  average /= buffer.size();

  ASSERT_GT(average, 0);
}

TEST(UnrealPlugin, TestCameraPoseConversion) {
  UnrealGuidelineOptions options;
  options.audio_frames_per_buffer = kFramesPerBuffer;
  std::unique_ptr<UnrealPlugin> plugin =
      absl::WrapUnique(CreateUnrealPlugin(options));

  plugin->Start();
  plugin->OnTracking(true);

  auto& impl = static_cast<UnrealPluginImpl&>(*plugin);

  std::optional<Transformation> camera_transformation;
  const motion::CameraMotionCallback callback =
      [&camera_transformation](
          const int64_t timestamp_us, const Transformation& world_t_camera,
          std::shared_ptr<camera::CameraModel> camera_model) {
        camera_transformation = world_t_camera;
      };
  impl.guideline_engine().motion_tracker().AddCameraMotionCallback(callback);

  plugin->OnCameraPose(100, 4, 5, 6, -0, -0, -0, 1);
  EXPECT_TRANSFORMATION_APPROX(
      *camera_transformation,
      Transformation({0, -0, M_SQRT1_2, M_SQRT1_2}, {5, 4, 6}), 1e-8);

  plugin->OnCameraPose(100, 4, 5, 6, 0, -M_SQRT1_2, M_SQRT1_2, 0);
  EXPECT_TRANSFORMATION_APPROX(*camera_transformation,
                               Transformation({0.5, 0.5, -0.5, 0.5}, {5, 4, 6}),
                               1e-8);

  plugin->OnCameraPose(100, 4, 5, 6, -M_SQRT1_2, 0, M_SQRT1_2, 0);
  EXPECT_TRANSFORMATION_APPROX(*camera_transformation,
                               Transformation({0, 1, -0, 0}, {5, 4, 6}), 1e-8);

  plugin->OnCameraPose(100, 4, 5, 6, -0.5, -0.5, -0.5, 0.5);
  EXPECT_TRANSFORMATION_APPROX(
      *camera_transformation,
      Transformation({M_SQRT1_2, -0, -0, -M_SQRT1_2}, {5, 4, 6}), 1e-8);
}

TEST(UnrealPlugin, TestGetProjectionMatrix) {
  UnrealGuidelineOptions options;
  options.audio_frames_per_buffer = kFramesPerBuffer;
  std::unique_ptr<UnrealPlugin> plugin =
      absl::WrapUnique(CreateUnrealPlugin(options));
  float projection[16];
  plugin->GetProjectionMatrix(1.0f, 100.0f, projection);
}

}  // namespace guideline::unreal
