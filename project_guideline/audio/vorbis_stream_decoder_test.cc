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

#include "project_guideline/audio/vorbis_stream_decoder.h"

#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "project_guideline/audio/audio_buffer.h"
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

TEST(VorbisStreamDecoder, DecodeFile) {
  auto ogg_contents = std::make_unique<std::string>();
  GL_ASSERT_OK(LoadTestOggFile(*ogg_contents));
  GL_ASSERT_OK_AND_ASSIGN(auto decoder,
                          VorbisStreamDecoder::Decode(std::move(ogg_contents)));

  ASSERT_EQ(198450, decoder->GetDecodedDurationInFrames());
  ASSERT_FALSE(decoder->IsAtEndOfStream());
  ASSERT_EQ(2, decoder->num_channels());
  ASSERT_EQ(44100, decoder->sample_rate_hz());
  ASSERT_EQ(0, decoder->GetNextFramePosition());

  PlanarAudioBuffer buffer;
  buffer.resize(decoder->num_channels());

  uint64_t total_frames_read = 0;
  const uint64_t kFramesReadSize = 1000;

  while (!decoder->IsAtEndOfStream()) {
    size_t frames_read = 0;
    GL_ASSERT_OK(decoder->GetNextBuffer(kFramesReadSize, buffer, frames_read));
    if (frames_read == 0) {
      break;
    }
    total_frames_read += frames_read;
  }

  ASSERT_TRUE(decoder->IsAtEndOfStream());
  ASSERT_EQ(decoder->GetDecodedDurationInFrames(), total_frames_read);
}

TEST(VorbisStreamDecoder, ReadFully) {
  auto ogg_contents = std::make_unique<std::string>();
  GL_ASSERT_OK(LoadTestOggFile(*ogg_contents));
  GL_ASSERT_OK_AND_ASSIGN(auto decoder,
                          VorbisStreamDecoder::Decode(std::move(ogg_contents)));

  GL_ASSERT_OK_AND_ASSIGN(auto buffer, decoder->ReadFully());
  ASSERT_EQ(2, buffer->size());
  ASSERT_EQ(decoder->GetDecodedDurationInFrames(), (*buffer)[0].size());
  ASSERT_EQ(decoder->GetDecodedDurationInFrames(), (*buffer)[1].size());
}

}  // namespace
}  // namespace guideline::audio
