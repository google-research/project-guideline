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

#include <algorithm>
#include <ios>
#include <string>

#include "absl/status/status.h"
#include "vorbis/codec.h"
#include "vorbis/vorbisenc.h"
#include "project_guideline/audio/audio_buffer.h"
#include "project_guideline/util/status.h"

namespace guideline::audio {

absl::StatusOr<std::unique_ptr<VorbisStreamEncoder>> VorbisStreamEncoder::Open(
    const std::string& filename, size_t num_channels, int sample_rate_hz,
    float quality) {
  auto ofs = std::make_unique<std::ofstream>();
  ofs->open(filename, std::ios::out | std::ios::trunc | std::ios::binary);

  if (!ofs->good()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Could not open output file: ", filename));
  }

  std::unique_ptr<VorbisStreamEncoder> encoder =
      absl::WrapUnique(new VorbisStreamEncoder(std::move(ofs), num_channels,
                                               sample_rate_hz, quality));
  GL_RETURN_IF_ERROR(encoder->Initialize());

  return encoder;
}

VorbisStreamEncoder::VorbisStreamEncoder(
    std::unique_ptr<std::ofstream> output_stream, size_t num_channels,
    int sample_rate_hz, float quality)
    : output_stream_(std::move(output_stream)),
      num_channels_(num_channels),
      sample_rate_hz_(sample_rate_hz),
      quality_(quality) {}

VorbisStreamEncoder::~VorbisStreamEncoder() { FlushAndClose().IgnoreError(); }

absl::Status VorbisStreamEncoder::Initialize() {
  vorbis_info_init(&vorbis_info_);
  if (vorbis_encode_init_vbr(&vorbis_info_, num_channels_, sample_rate_hz_,
                             quality_) != 0) {
    return absl::InternalError("Failed to initialize vorbis encoder");
  }

  vorbis_comment_init(&vorbis_comment_);
  vorbis_comment_add_tag(&vorbis_comment_, "ENCODER", "Guideline");
  vorbis_analysis_init(&vorbis_state_, &vorbis_info_);
  vorbis_block_init(&vorbis_state_, &vorbis_block_);
  ogg_stream_init(&stream_state_, /*serial_number=*/1);

  ogg_packet header;
  ogg_packet header_comments;
  ogg_packet header_code;
  vorbis_analysis_headerout(&vorbis_state_, &vorbis_comment_, &header,
                            &header_comments, &header_code);
  ogg_stream_packetin(&stream_state_, &header);
  ogg_stream_packetin(&stream_state_, &header_comments);
  ogg_stream_packetin(&stream_state_, &header_code);

  while (true) {
    if (ogg_stream_flush(&stream_state_, &ogg_page_) == 0) {
      break;
    }
    GL_RETURN_IF_ERROR(WriteOggPage());
  }
  opened_ = true;
  return absl::OkStatus();
}

absl::Status VorbisStreamEncoder::AddPlanarBuffer(
    const PlanarAudioBuffer& buffer) {
  CHECK(opened_.load());

  std::vector<const float*> channel_ptrs(buffer.size());
  for (size_t channel = 0; channel < channel_ptrs.size(); ++channel) {
    channel_ptrs[channel] = buffer[channel].data();
  }

  PrepareVorbisBuffer(channel_ptrs.data(), buffer.size(), buffer[0].size());
  return PerformEncoding();
}

absl::Status VorbisStreamEncoder::AddInterleavedBuffer(int16_t* buffer_ptr,
                                                       size_t num_channels,
                                                       size_t num_frames) {
  if (conversion_buffer_.capacity() != num_channels) {
    conversion_buffer_.resize(num_channels);
  }

  for (size_t channel = 0; channel < num_channels; ++channel) {
    if (conversion_buffer_[channel].size() != num_frames) {
      conversion_buffer_[channel].resize(num_frames);
    }
  }

  size_t num_samples = num_channels * num_frames;
  for (size_t channel = 0; channel < num_channels; ++channel) {
    float* output_ptr = &conversion_buffer_[channel][0];
    size_t frame_index = 0;
    for (size_t sample_index = channel; sample_index < num_samples;
         sample_index += num_channels) {
      ConvertSampleToFloatFormat(buffer_ptr[sample_index],
                                 &output_ptr[frame_index++]);
    }
  }

  output_buffer_channel_ptrs_.resize(num_channels);
  for (size_t channel = 0; channel < num_channels; ++channel) {
    output_buffer_channel_ptrs_[channel] = &conversion_buffer_[channel][0];
  }

  PrepareVorbisBuffer(output_buffer_channel_ptrs_.data(), num_channels,
                      num_frames);

  return PerformEncoding();
}

absl::Status VorbisStreamEncoder::FlushAndClose() {
  if (!opened_.load()) {
    return absl::OkStatus();
  }

  vorbis_analysis_wrote(&vorbis_state_, 0);
  GL_RETURN_IF_ERROR(PerformEncoding());

  output_stream_->close();

  vorbis_comment_clear(&vorbis_comment_);
  vorbis_dsp_clear(&vorbis_state_);
  vorbis_block_clear(&vorbis_block_);
  ogg_stream_clear(&stream_state_);
  vorbis_info_clear(&vorbis_info_);

  opened_ = false;

  return absl::OkStatus();
}

void VorbisStreamEncoder::PrepareVorbisBuffer(const float* const* input_ptrs,
                                              size_t num_channels,
                                              size_t num_frames) {
  float** buffer = vorbis_analysis_buffer(
      &vorbis_state_, static_cast<int>(num_channels * num_frames));
  for (size_t channel = 0; channel < num_channels; ++channel) {
    std::copy_n(input_ptrs[channel], num_frames, buffer[channel]);
  }

  vorbis_analysis_wrote(&vorbis_state_, static_cast<int>(num_frames));
}

absl::Status VorbisStreamEncoder::PerformEncoding() {
  CHECK(opened_.load());
  while (vorbis_analysis_blockout(&vorbis_state_, &vorbis_block_) == 1) {
    vorbis_analysis(&vorbis_block_, nullptr);
    vorbis_bitrate_addblock(&vorbis_block_);

    while (vorbis_bitrate_flushpacket(&vorbis_state_, &ogg_packet_)) {
      ogg_stream_packetin(&stream_state_, &ogg_packet_);

      bool end_of_stream = false;
      while (!end_of_stream) {
        int result = ogg_stream_pageout(&stream_state_, &ogg_page_);
        if (result == 0) {
          break;
        }
        GL_RETURN_IF_ERROR(WriteOggPage());
        if (ogg_page_eos(&ogg_page_)) {
          end_of_stream = true;
        }
      }
    }
  }
  return absl::OkStatus();
}

absl::Status VorbisStreamEncoder::WriteOggPage() {
  output_stream_->write(reinterpret_cast<char*>(ogg_page_.header),
                        ogg_page_.header_len);
  output_stream_->write(reinterpret_cast<char*>(ogg_page_.body),
                        ogg_page_.body_len);
  if (!output_stream_->good()) {
    return absl::InternalError("Failed to write OGG page");
  }
  return absl::OkStatus();
}

}  // namespace guideline::audio
