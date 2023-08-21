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

#include "project_guideline/audio/resampler.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "project_guideline/audio/audio_buffer.h"
#include "project_guideline/audio/vorbis_stream_decoder.h"
#include "project_guideline/audio/vorbis_stream_encoder.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/file.h"

namespace guideline::audio {
namespace {

const char kTestOggFile[] =
    "/project_guideline/project_guideline/audio/testdata/song.ogg";

absl::Status LoadTestOggFile(std::string& contents) {
  auto ogg_file = std::string(getenv("TEST_SRCDIR")) + kTestOggFile;
  return util::ReadFileToString(ogg_file, contents);
}

TEST(Resampler, UpSample) {
  auto ogg_contents = std::make_unique<std::string>();
  GL_ASSERT_OK(LoadTestOggFile(*ogg_contents));
  GL_ASSERT_OK_AND_ASSIGN(auto decoder,
                          VorbisStreamDecoder::Decode(std::move(ogg_contents)));
  ASSERT_EQ(decoder->sample_rate_hz(), 44100);
  ASSERT_EQ(decoder->num_channels(), 2);

  GL_ASSERT_OK_AND_ASSIGN(auto audio, decoder->ReadFully());

  Resampler resampler(decoder->sample_rate_hz(), 48000);
  PlanarAudioBuffer buffer(2);
  resampler.Resample((*audio)[0], buffer[0]);
  resampler.Resample((*audio)[1], buffer[1]);

  // Encode the resampled audio file which can be manually inspected for
  // audio artifacts / correctness.
  GL_ASSERT_OK_AND_ASSIGN(
      auto encoder, VorbisStreamEncoder::Open(
                        std::string(getenv("TEST_TMPDIR")) + "/upsampled.ogg", 2,
                        resampler.output_sample_rate_hz(), /*quality=*/1.0f));
  GL_ASSERT_OK(encoder->AddPlanarBuffer(buffer));
  GL_ASSERT_OK(encoder->FlushAndClose());
}

TEST(Resampler, DownSample) {
  auto ogg_contents = std::make_unique<std::string>();
  GL_ASSERT_OK(LoadTestOggFile(*ogg_contents));
  GL_ASSERT_OK_AND_ASSIGN(auto decoder,
                          VorbisStreamDecoder::Decode(std::move(ogg_contents)));
  ASSERT_EQ(decoder->sample_rate_hz(), 44100);
  ASSERT_EQ(decoder->num_channels(), 2);

  GL_ASSERT_OK_AND_ASSIGN(auto audio, decoder->ReadFully());

  Resampler resampler(decoder->sample_rate_hz(), 20000);
  PlanarAudioBuffer buffer(2);
  resampler.Resample((*audio)[0], buffer[0]);
  resampler.Resample((*audio)[1], buffer[1]);

  // Encode the resampled audio file which can be manually inspected for
  // audio artifacts / correctness.
  GL_ASSERT_OK_AND_ASSIGN(
      auto encoder,
      VorbisStreamEncoder::Open(
          std::string(getenv("TEST_TMPDIR")) + "/downsampled.ogg", 2,
          resampler.output_sample_rate_hz(), /*quality=*/1.0f));
  GL_ASSERT_OK(encoder->AddPlanarBuffer(buffer));
  GL_ASSERT_OK(encoder->FlushAndClose());
}

TEST(Resampler, ResampleChunk) {
  auto ogg_contents = std::make_unique<std::string>();
  GL_ASSERT_OK(LoadTestOggFile(*ogg_contents));
  GL_ASSERT_OK_AND_ASSIGN(auto decoder,
                          VorbisStreamDecoder::Decode(std::move(ogg_contents)));
  ASSERT_EQ(decoder->sample_rate_hz(), 44100);
  ASSERT_EQ(decoder->num_channels(), 2);

  size_t frames_4 = decoder->GetDecodedDurationInFrames() / 4;

  GL_ASSERT_OK_AND_ASSIGN(auto audio, decoder->ReadFully());

  PlanarAudioBuffer buffer(2);
  buffer[0].resize(decoder->GetDecodedDurationInFrames() * 1.25);
  buffer[1].resize(decoder->GetDecodedDurationInFrames() * 1.25);

  // First 1/4 of audio normal speed.
  size_t output_index = 0;
  size_t input_index = 0;
  float playback_rate = 1.0f;
  size_t num_frames = frames_4;
  size_t num_frames_copied =
      Resampler::ResampleChunk((*audio)[0], buffer[0], playback_rate,
                               input_index, output_index, num_frames);
  ASSERT_EQ(num_frames_copied, num_frames);
  num_frames_copied =
      Resampler::ResampleChunk((*audio)[1], buffer[1], playback_rate,
                               input_index, output_index, num_frames);
  ASSERT_EQ(num_frames_copied, num_frames);

  // Next 1/4 of audio sped up (pitch increase).
  output_index += num_frames_copied;
  input_index = frames_4;
  playback_rate = 1.2f;
  num_frames = frames_4 / playback_rate;
  num_frames_copied =
      Resampler::ResampleChunk((*audio)[0], buffer[0], playback_rate,
                               input_index, output_index, num_frames);
  ASSERT_EQ(num_frames_copied, num_frames);
  num_frames_copied =
      Resampler::ResampleChunk((*audio)[1], buffer[1], playback_rate,
                               input_index, output_index, num_frames);
  ASSERT_EQ(num_frames_copied, num_frames);

  // Next 1/4 of audio slowed down (pitch decrease).
  output_index += num_frames_copied;
  input_index = frames_4 * 2;
  playback_rate = 0.7f;
  num_frames = frames_4 / playback_rate;
  num_frames_copied =
      Resampler::ResampleChunk((*audio)[0], buffer[0], playback_rate,
                               input_index, output_index, num_frames);
  ASSERT_EQ(num_frames_copied, num_frames);
  num_frames_copied =
      Resampler::ResampleChunk((*audio)[1], buffer[1], playback_rate,
                               input_index, output_index, num_frames);
  ASSERT_EQ(num_frames_copied, num_frames);

  // Final 1/4 of audio normal speed
  output_index += num_frames_copied;
  input_index = frames_4 * 3;
  playback_rate = 1.0f;
  num_frames = frames_4 / playback_rate;
  num_frames_copied =
      Resampler::ResampleChunk((*audio)[0], buffer[0], playback_rate,
                               input_index, output_index, num_frames);
  ASSERT_EQ(num_frames_copied, num_frames);
  num_frames_copied =
      Resampler::ResampleChunk((*audio)[1], buffer[1], playback_rate,
                               input_index, output_index, num_frames);
  ASSERT_EQ(num_frames_copied, num_frames);

  // Encode the resampled audio file which can be manually inspected for
  // audio artifacts / correctness.
  GL_ASSERT_OK_AND_ASSIGN(
      auto encoder,
      VorbisStreamEncoder::Open(
          std::string(getenv("TEST_TMPDIR")) + "/chunkresampled.ogg", 2,
          decoder->sample_rate_hz(), /*quality=*/1.0f));
  GL_ASSERT_OK(encoder->AddPlanarBuffer(buffer));
  GL_ASSERT_OK(encoder->FlushAndClose());
}

}  // namespace
}  // namespace guideline::audio
