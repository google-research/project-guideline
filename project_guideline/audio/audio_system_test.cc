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

#include "project_guideline/audio/audio_system.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>  // NOLINT
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "absl/time/time.h"
#include "project_guideline/audio/audio_buffer.h"
#include "project_guideline/audio/audio_output_stream.h"
#include "project_guideline/audio/vorbis_stream_decoder.h"
#include "project_guideline/environment/control_signal.h"
#include "project_guideline/logging/noop_guideline_logger.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/file.h"

namespace guideline::audio {
namespace {

using audio::PlanarAudioBuffer;
using audio::VorbisStreamDecoder;

static const size_t kNumChannels = 2;
static const size_t kFramesPerBuffer = 8192;
static const int kSampleRate = 48000;
static const float kSampleTolerance = 0.01;

class FakeAudioOut : public audio::AudioOutputStream {
 public:
  FakeAudioOut()
      : audio::AudioOutputStream(kSampleRate, kFramesPerBuffer, kNumChannels) {
    audio_output_buffer_.resize(GetNumChannels() * GetFramesPerBuffer());
  }

  ~FakeAudioOut() override { StopPlayback().IgnoreError(); }

  absl::Status StartPlayback(
      const audio::AudioOutputStream::Callback& callback) override {
    callback_ = callback;
    return absl::OkStatus();
  }
  absl::Status StopPlayback() override {
    callback_ = std::nullopt;
    return absl::OkStatus();
  }
  bool GetNextOutputBuffer(int16_t* output_ptr) {
    CHECK(callback_.has_value());
    return (*callback_)(output_ptr, kNumChannels, kFramesPerBuffer);
  }

 private:
  std::optional<audio::AudioOutputStream::Callback> callback_ = std::nullopt;
  std::vector<int16_t> audio_output_buffer_;
};

bool ReadAudio(FakeAudioOut& audio_out, std::vector<int16_t>& buffer,
               absl::Duration at_least_duration) {
  double frames = absl::ToDoubleSeconds(at_least_duration) * kSampleRate;
  size_t num_buffers = std::ceil(frames / kFramesPerBuffer);

  buffer.resize(num_buffers * kFramesPerBuffer * kNumChannels);

  for (size_t i = 0; i < num_buffers; ++i) {
    if (!audio_out.GetNextOutputBuffer(&buffer[i * kFramesPerBuffer])) {
      return false;
    }
  }

  return true;
}

std::vector<float> AveragePower(std::vector<int16_t>& buffer) {
  std::vector<float> averages;
  averages.resize(kNumChannels);

  size_t num_frames = buffer.size() / kNumChannels;
  for (size_t frame = 0; frame < num_frames; ++frame) {
    size_t sample_idx = frame * kNumChannels;
    for (size_t channel = 0; channel < kNumChannels; ++channel) {
      averages[channel] += std::abs(buffer[sample_idx + channel]);
    }
  }

  for (size_t channel = 0; channel < kNumChannels; ++channel) {
    averages[channel] /= kFramesPerBuffer;
    audio::ConvertSampleToFloatFormat(averages[channel], &averages[channel]);
  }

  return averages;
}

std::vector<float> AveragePower(PlanarAudioBuffer& buffer) {
  std::vector<float> averages;
  size_t num_channels = buffer.size();
  averages.resize(num_channels);

  size_t num_frames = buffer[0].size() / buffer.size();
  for (size_t frame = 0; frame < num_frames; ++frame) {
    for (size_t channel = 0; channel < num_channels; ++channel) {
      averages[channel] += std::abs(buffer[channel][frame]);
    }
  }

  for (size_t channel = 0; channel < num_channels; ++channel) {
    averages[channel] /= kFramesPerBuffer;
  }

  return averages;
}

void GoToGuidingState(FakeAudioOut& audio_out, AudioSystem& audio_system) {
  environment::ControlSignal signal;
  signal.lateral_movement_meters = 0.;
  signal.rotation_movement_degrees = 0.;

  std::vector<int16_t> buffer;
  // INITIALIZING
  ASSERT_TRUE(ReadAudio(audio_out, buffer, absl::Seconds(3)));

  // READY
  audio_system.OnControlSignal(signal);
  ASSERT_TRUE(ReadAudio(audio_out, buffer, absl::Seconds(1)));

  // READY -> GUIDING
  audio_system.OnControlSignal(signal);

  // There is no audio after the ready sound finishes, until the next control
  // signal is received.
  ASSERT_FALSE(ReadAudio(audio_out, buffer, absl::Seconds(1)));

  audio_system.OnControlSignal(signal);
}

TEST(AudioSystem, TestAudio) {
  auto audio_out = std::make_shared<FakeAudioOut>();
  GL_ASSERT_OK_AND_ASSIGN(
      auto audio_system,
      AudioSystem::Create(AudioSystemOptions::default_instance(),
                          std::make_unique<logging::NoopGuidelineLogger>(),
                          audio_out));

  const std::string tmp_output_file =
      std::filesystem::path(std::string(getenv("TEST_TMPDIR")))
          .append("output.ogg");
  audio_system->RecordToFile(tmp_output_file);
  audio_system->EnableRecording();

  GL_ASSERT_OK(audio_system->Start());

  GoToGuidingState(*audio_out, *audio_system);

  std::vector<int16_t> buffer;
  ReadAudio(*audio_out, buffer, absl::Seconds(4));
  GL_ASSERT_OK(audio_system->Stop());

  std::vector<float> average_power = AveragePower(buffer);
  ASSERT_EQ(kNumChannels, average_power.size());
  ASSERT_NEAR(0.3, average_power[0], kSampleTolerance);
  ASSERT_NEAR(0.3, average_power[1], kSampleTolerance);
}

TEST(AudioSystem, TestLateralMovement) {
  auto audio_out = std::make_shared<FakeAudioOut>();
  GL_ASSERT_OK_AND_ASSIGN(
      auto audio_system,
      AudioSystem::Create(AudioSystemOptions::default_instance(),
                          std::make_unique<logging::NoopGuidelineLogger>(),
                          audio_out));
  GL_ASSERT_OK(audio_system->Start());

  GoToGuidingState(*audio_out, *audio_system);

  environment::ControlSignal signal;
  signal.lateral_movement_meters = -1.;
  signal.rotation_movement_degrees = 0.;
  audio_system->OnControlSignal(signal);

  std::vector<int16_t> buffer;
  ReadAudio(*audio_out, buffer, absl::Seconds(4));

  // Audio is mostly in left channel.
  std::vector<float> average_power = AveragePower(buffer);
  ASSERT_GT(average_power[0] - average_power[1], 0.15);

  signal.lateral_movement_meters = 1.;
  audio_system->OnControlSignal(signal);

  ASSERT_TRUE(ReadAudio(*audio_out, buffer, absl::Seconds(4)));

  // Audio is mostly in right channel.
  average_power = AveragePower(buffer);
  ASSERT_GT(average_power[1] - average_power[0], 0.15);

  GL_ASSERT_OK(audio_system->Stop());
}

TEST(AudioSystem, TestWarningSound) {
  auto audio_out = std::make_shared<FakeAudioOut>();
  GL_ASSERT_OK_AND_ASSIGN(
      auto audio_system,
      AudioSystem::Create(AudioSystemOptions::default_instance(),
                          std::make_unique<logging::NoopGuidelineLogger>(),
                          audio_out));
  GL_ASSERT_OK(audio_system->Start());

  GoToGuidingState(*audio_out, *audio_system);

  environment::ControlSignal signal;
  signal.lateral_movement_meters = -5.;
  signal.rotation_movement_degrees = 0.;
  audio_system->OnControlSignal(signal);

  std::vector<int16_t> buffer;
  ReadAudio(*audio_out, buffer, absl::Seconds(4));

  // Warning audio is mostly in right channel.
  std::vector<float> average_power = AveragePower(buffer);
  ASSERT_GT(average_power[1] - average_power[0], 0.1);

  signal.lateral_movement_meters = 5.;
  audio_system->OnControlSignal(signal);

  ASSERT_TRUE(ReadAudio(*audio_out, buffer, absl::Seconds(4)));

  // Warning audio is mostly in left channel.
  average_power = AveragePower(buffer);
  ASSERT_GT(average_power[0] - average_power[1], 0.1);

  GL_ASSERT_OK(audio_system->Stop());
}

TEST(AudioSystem, WaitForAudioStream) {
  auto audio_out = std::make_shared<FakeAudioOut>();
  GL_ASSERT_OK_AND_ASSIGN(
      auto audio_system,
      AudioSystem::Create(AudioSystemOptions::default_instance(),
                          std::make_unique<logging::NoopGuidelineLogger>(),
                          audio_out));
  GL_ASSERT_OK(audio_system->Start());

  std::vector<int16_t> buffer;
  buffer.resize(kFramesPerBuffer * kNumChannels);
  ASSERT_TRUE(audio_out->GetNextOutputBuffer(&buffer[0]));

  GL_ASSERT_OK(audio_system->WaitForAudioStream());

  GL_ASSERT_OK(audio_system->Stop());
}

TEST(AudioSystem, RecordAudio) {
  auto audio_out = std::make_shared<FakeAudioOut>();
  GL_ASSERT_OK_AND_ASSIGN(
      auto audio_system,
      AudioSystem::Create(AudioSystemOptions::default_instance(),
                          std::make_unique<logging::NoopGuidelineLogger>(),
                          audio_out));

  const std::string tmp_output_file =
      std::filesystem::path(std::string(getenv("TEST_TMPDIR")))
          .append("output.ogg");
  audio_system->RecordToFile(tmp_output_file);

  GL_ASSERT_OK(audio_system->Start());

  environment::ControlSignal signal;
  signal.lateral_movement_meters = 0.;
  signal.rotation_movement_degrees = 0.;
  audio_system->OnControlSignal(signal);

  // Recording will not contain this first 4 seconds before EnableRecording().
  std::vector<int16_t> buffer;
  ASSERT_TRUE(ReadAudio(*audio_out, buffer, absl::Seconds(4)));

  audio_system->EnableRecording();
  ASSERT_TRUE(ReadAudio(*audio_out, buffer, absl::Seconds(4)));

  GL_ASSERT_OK(audio_system->Stop());

  auto ogg_contents = std::make_unique<std::string>();
  GL_ASSERT_OK(util::ReadFileToString(tmp_output_file, *ogg_contents));

  GL_ASSERT_OK_AND_ASSIGN(auto decoder,
                          VorbisStreamDecoder::Decode(std::move(ogg_contents)));
  ASSERT_EQ(4, (size_t)(decoder->GetDecodedDurationInFrames() / kSampleRate));
  ASSERT_EQ(kNumChannels, decoder->num_channels());
  ASSERT_EQ(kSampleRate, decoder->sample_rate_hz());

  GL_ASSERT_OK_AND_ASSIGN(auto decoded_buffer, decoder->ReadFully());
  std::vector<float> average_power = AveragePower(*decoded_buffer);
  ASSERT_GT(average_power[0], 0.04);
  ASSERT_GT(average_power[1], 0.04);
}

TEST(AudioSystem, LegacySoundPack) {
  auto audio_out = std::make_shared<FakeAudioOut>();
  AudioSystemOptions options;
  options.mutable_legacy_sound_pack_options()->set_type(
      LegacySoundPackOptions::V4_2);
  GL_ASSERT_OK_AND_ASSIGN(
      auto audio_system,
      AudioSystem::Create(options,
                          std::make_unique<logging::NoopGuidelineLogger>(),
                          audio_out));
  GL_ASSERT_OK(audio_system->Start());

  GoToGuidingState(*audio_out, *audio_system);

  environment::ControlSignal signal;
  signal.target_rotation_movement_degrees = -45;
  audio_system->OnControlSignal(signal);

  std::vector<int16_t> buffer;
  ReadAudio(*audio_out, buffer, absl::Seconds(4));

  // Audio is mostly in left channel.
  std::vector<float> average_power = AveragePower(buffer);
  ASSERT_GT(average_power[0] - average_power[1], 0.12);

  signal.target_rotation_movement_degrees = 45;
  audio_system->OnControlSignal(signal);

  ASSERT_TRUE(ReadAudio(*audio_out, buffer, absl::Seconds(4)));

  // Audio is mostly in right channel.
  average_power = AveragePower(buffer);
  ASSERT_GT(average_power[1] - average_power[0], 0.12);

  GL_ASSERT_OK(audio_system->Stop());
}

}  // namespace
}  // namespace guideline::audio
