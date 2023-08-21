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

#ifndef PROJECT_GUIDELINE_AUDIO_VORBIS_STREAM_DECODER_H_
#define PROJECT_GUIDELINE_AUDIO_VORBIS_STREAM_DECODER_H_

#define OV_EXCLUDE_STATIC_CALLBACKS

#include <atomic>

#include "absl/status/statusor.h"
#include "vorbis/vorbisfile.h"
#include "project_guideline/audio/audio_buffer.h"

namespace guideline::audio {

// Decodes Ogg Vorbis audio streams.
// This is intended for smaller audio files where the entire contents of the
// file is loaded into memory.
class VorbisStreamDecoder {
 public:
  ~VorbisStreamDecoder();

  static absl::StatusOr<std::unique_ptr<VorbisStreamDecoder>> Decode(
      std::unique_ptr<std::string> ogg_vorbis_data);

  absl::StatusOr<std::unique_ptr<PlanarAudioBuffer>> ReadFully();
  absl::Status GetNextBuffer(size_t num_frames,
                             PlanarAudioBuffer& output_buffer,
                             size_t& frames_read);

  size_t GetNextFramePosition();
  size_t GetDecodedDurationInFrames();
  bool IsAtEndOfStream() const;

  size_t num_channels() const { return num_channels_; }

  int sample_rate_hz() const { return sample_rate_hz_; }

  struct MemoryStreamDescriptor {
    const unsigned char* data = nullptr;
    uint64_t data_size = 0;
    uint64_t data_offset = 0;
  };

 private:
  explicit VorbisStreamDecoder(std::unique_ptr<std::string> ogg_vorbis_data);

  absl::Status Initialize();

  std::unique_ptr<std::string> data_;
  MemoryStreamDescriptor memory_descriptor_;

  size_t num_channels_;
  int sample_rate_hz_;

  OggVorbis_File vorbis_file_;
  size_t buffer_frames_;

  std::atomic<bool> end_of_stream_;
};
}  // namespace guideline::audio

#endif  // PROJECT_GUIDELINE_AUDIO_VORBIS_STREAM_DECODER_H_
