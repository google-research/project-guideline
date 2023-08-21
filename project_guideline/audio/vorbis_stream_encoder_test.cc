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

#include "project_guideline/audio/vorbis_stream_encoder.h"

#include <filesystem>  // NOLINT

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "project_guideline/audio/vorbis_stream_decoder.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/file.h"

namespace guideline::audio {
namespace {

const float kInt16Min = static_cast<float>(-0x7FFF);
const float kInt16Max = static_cast<float>(0x7FFF);

TEST(VorbisStreamEncoder, EncodeInterleaved) {
  const std::string tmp_output_file =
      std::filesystem::path(std::string(getenv("TEST_TMPDIR")))
          .append("output.ogg");

  static const size_t kNumChannels = 2;
  static const int kSampleRate = 48000;
  static const float kQuality = 0.8f;
  static const float kNumFrames = 1024;

  GL_ASSERT_OK_AND_ASSIGN(
      auto encoder, VorbisStreamEncoder::Open(tmp_output_file, kNumChannels,
                                              kSampleRate, kQuality));

  std::vector<int16_t> buffer;
  const size_t num_samples = kNumFrames * kNumChannels;
  buffer.resize(num_samples);

  for (size_t frame = 0; frame < kNumFrames; ++frame) {
    size_t sample_idx = frame * kNumChannels;

    if (frame < kNumFrames / 2) {
      // For the first half the right channel has max value.
      buffer[sample_idx] = 0;
      buffer[sample_idx + 1] = kInt16Max;
    } else {
      // For the second half the left channel has min value.
      buffer[sample_idx] = kInt16Min;
      buffer[sample_idx + 1] = 0;
    }
  }

  // Write the buffer twice to repeat the sequence.
  GL_ASSERT_OK(
      encoder->AddInterleavedBuffer(buffer.data(), kNumChannels, kNumFrames));
  GL_ASSERT_OK(
      encoder->AddInterleavedBuffer(buffer.data(), kNumChannels, kNumFrames));

  GL_ASSERT_OK(encoder->FlushAndClose());

  auto ogg_contents = std::make_unique<std::string>();
  GL_ASSERT_OK(util::ReadFileToString(tmp_output_file, *ogg_contents));

  GL_ASSERT_OK_AND_ASSIGN(auto decoder,
                          VorbisStreamDecoder::Decode(std::move(ogg_contents)));

  ASSERT_EQ(kNumFrames * 2, decoder->GetDecodedDurationInFrames());
  ASSERT_EQ(kNumChannels, decoder->num_channels());
  ASSERT_EQ(kSampleRate, decoder->sample_rate_hz());

  GL_ASSERT_OK_AND_ASSIGN(auto read, decoder->ReadFully());
  ASSERT_EQ(2, read->size());

  size_t duration_in_frames = decoder->GetDecodedDurationInFrames();
  ASSERT_EQ(duration_in_frames, (*read)[0].size());
  ASSERT_EQ(duration_in_frames, (*read)[1].size());

  static const float kTolerance = 0.02f;

  // Check that 1st quarter of the file has [0, 1]
  EXPECT_NEAR(0, (*read)[0][256], kTolerance);
  EXPECT_NEAR(1, (*read)[1][256], kTolerance);

  // Check that 2nd quarter of the file has [-1, 0]
  EXPECT_NEAR(-1, (*read)[0][768], kTolerance);
  EXPECT_NEAR(0, (*read)[1][768], kTolerance);

  // The stream repeats after sample 1024
  EXPECT_NEAR(0, (*read)[0][1280], kTolerance);
  EXPECT_NEAR(1, (*read)[1][1280], kTolerance);

  EXPECT_NEAR(-1, (*read)[0][1792], kTolerance);
  EXPECT_NEAR(0, (*read)[1][1792], kTolerance);
}

}  // namespace
}  // namespace guideline::audio
