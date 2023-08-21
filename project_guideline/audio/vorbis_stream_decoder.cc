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

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "project_guideline/util/status.h"

namespace guideline::audio {

namespace {
typedef VorbisStreamDecoder::MemoryStreamDescriptor MemoryStreamDescriptor;
typedef long vorbis_long;  // NOLINT

size_t VorbisReadCallback(void* pointer, size_t member_size,
                          size_t number_members, void* datasource) {
  MemoryStreamDescriptor* memory_stream =
      reinterpret_cast<MemoryStreamDescriptor*>(datasource);
  uint64_t copy_size =
      std::min(static_cast<uint64_t>(member_size * number_members),
               memory_stream->data_size - memory_stream->data_offset);
  std::memcpy(pointer, memory_stream->data + memory_stream->data_offset,
              static_cast<size_t>(copy_size));
  memory_stream->data_offset += copy_size;
  return static_cast<size_t>(copy_size);
}

int VorbisSeekCallback(void* datasource, ogg_int64_t offset, int origin) {
  MemoryStreamDescriptor* memory_stream =
      reinterpret_cast<MemoryStreamDescriptor*>(datasource);
  switch (origin) {
    case SEEK_SET:
      memory_stream->data_offset = 0;
      break;
    case SEEK_CUR:
      break;
    case SEEK_END:
      memory_stream->data_offset = memory_stream->data_size;
      break;
    default:
      return 1;
      break;
  }

  memory_stream->data_offset += offset;
  return 0;
}

int VorbisCloseCallback(void* datasource) { return 0; }

vorbis_long VorbisTellCallback(void* datasource) {
  MemoryStreamDescriptor* memory_stream =
      reinterpret_cast<MemoryStreamDescriptor*>(datasource);
  return static_cast<vorbis_long>(memory_stream->data_offset);
}
}  // namespace

absl::StatusOr<std::unique_ptr<VorbisStreamDecoder>>
VorbisStreamDecoder::Decode(std::unique_ptr<std::string> ogg_vorbis_data) {
  std::unique_ptr<VorbisStreamDecoder> decoder =
      absl::WrapUnique(new VorbisStreamDecoder(std::move(ogg_vorbis_data)));
  GL_RETURN_IF_ERROR(decoder->Initialize());
  return decoder;
}

VorbisStreamDecoder::VorbisStreamDecoder(std::unique_ptr<std::string> data)
    : data_(std::move(data)),
      num_channels_(0),
      sample_rate_hz_(-1),
      buffer_frames_(1024),
      end_of_stream_(false) {
  memset(&vorbis_file_, 0, sizeof(vorbis_file_));
}

VorbisStreamDecoder::~VorbisStreamDecoder() { ov_clear(&vorbis_file_); }

absl::Status VorbisStreamDecoder::Initialize() {
  ov_callbacks vorbis_callbacks = {&VorbisReadCallback, &VorbisSeekCallback,
                                   &VorbisCloseCallback, &VorbisTellCallback};

  memory_descriptor_.data = (const unsigned char*)data_->c_str();
  memory_descriptor_.data_size = data_->size();
  memory_descriptor_.data_offset = 0;

  int result = ov_open_callbacks(reinterpret_cast<void*>(&memory_descriptor_),
                                 &vorbis_file_, nullptr, 0, vorbis_callbacks);
  if (result != 0) {
    memset(&memory_descriptor_, 0, sizeof(memory_descriptor_));
    return absl::InternalError("Failed to initialize Ogg Vorbis");
  }

  vorbis_info* file_info = ov_info(&vorbis_file_, -1);
  num_channels_ = static_cast<size_t>(file_info->channels);
  sample_rate_hz_ = static_cast<int>(file_info->rate);

  return absl::OkStatus();
}

size_t VorbisStreamDecoder::GetNextFramePosition() {
  return static_cast<size_t>(ov_pcm_tell(&vorbis_file_));
}

size_t VorbisStreamDecoder::GetDecodedDurationInFrames() {
  int64_t duration = static_cast<int64_t>(ov_pcm_total(&vorbis_file_, -1));
  if (duration == OV_EINVAL) {
    return 0;
  }
  return static_cast<size_t>(duration);
}

absl::StatusOr<std::unique_ptr<PlanarAudioBuffer>>
VorbisStreamDecoder::ReadFully() {
  auto decoded_duration_frames = GetDecodedDurationInFrames();

  auto planar_pcm_data = std::make_unique<PlanarAudioBuffer>();
  planar_pcm_data->resize(num_channels_);
  for (size_t channel = 0; channel < num_channels_; ++channel) {
    (*planar_pcm_data)[channel].reserve(
        static_cast<size_t>(decoded_duration_frames));
  }

  size_t read_frames = 0;
  GL_RETURN_IF_ERROR(
      GetNextBuffer(decoded_duration_frames, *planar_pcm_data, read_frames));

  if (read_frames != decoded_duration_frames) {
    return absl::InternalError("Failed to read entire vorbis file");
  }

  return planar_pcm_data;
}

absl::Status VorbisStreamDecoder::GetNextBuffer(
    size_t num_frames, PlanarAudioBuffer& output_buffer, size_t& frames_read) {
  size_t total_frames_decoded = 0;
  const size_t destination_buffer_frames = num_frames;
  while (total_frames_decoded < destination_buffer_frames) {
    float** pcm_out = nullptr;
    int section = 0;
    const size_t target_frame_count = std::min(
        buffer_frames_, destination_buffer_frames - total_frames_decoded);
    const vorbis_long decoded_frames =
        ov_read_float(&vorbis_file_, &pcm_out,
                      static_cast<int>(target_frame_count), &section);
    if (decoded_frames > 0) {
      for (size_t channel = 0; channel < num_channels_; ++channel) {
        output_buffer[channel].insert(output_buffer[channel].end(),
                                      pcm_out[channel],
                                      pcm_out[channel] + decoded_frames);
      }
    } else if (decoded_frames < 0) {
      end_of_stream_ = true;
      return absl::InternalError("Error decoding OggVorbis file");
    } else {
      end_of_stream_ = true;
      break;
    }

    total_frames_decoded += decoded_frames;
  }

  frames_read = total_frames_decoded;
  return absl::OkStatus();
}

bool VorbisStreamDecoder::IsAtEndOfStream() const {
  return end_of_stream_.load();
}

}  // namespace guideline::audio
