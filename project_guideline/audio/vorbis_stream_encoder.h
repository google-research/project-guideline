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

#ifndef PROJECT_GUIDELINE_AUDIO_VORBIS_STREAM_ENCODER_H_
#define PROJECT_GUIDELINE_AUDIO_VORBIS_STREAM_ENCODER_H_

#include <fstream>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "ogg/ogg.h"
#include "vorbis/codec.h"
#include "project_guideline/audio/audio_buffer.h"

namespace guideline::audio {

// Utility for encoding an audio stream to an Ogg Vorbis file.
class VorbisStreamEncoder {
 public:
  static absl::StatusOr<std::unique_ptr<VorbisStreamEncoder>> Open(
      const std::string& filename, size_t num_channels, int sample_rate_hz,
      float quality);

  ~VorbisStreamEncoder();

  absl::Status AddPlanarBuffer(const PlanarAudioBuffer& buffer);

  absl::Status AddInterleavedBuffer(int16_t* buffer_ptr, size_t num_channels,
                                    size_t num_frames);

  absl::Status FlushAndClose();

 private:
  explicit VorbisStreamEncoder(std::unique_ptr<std::ofstream> output_stream,
                               size_t num_channels, int sample_rate_hz,
                               float quality);

  absl::Status Initialize();

  void PrepareVorbisBuffer(const float* const* input_ptrs, size_t num_channels,
                           size_t num_frames);

  absl::Status PerformEncoding();

  absl::Status WriteOggPage();

  std::unique_ptr<std::ofstream> output_stream_;
  const size_t num_channels_;
  const int sample_rate_hz_;
  const float quality_;

  std::atomic<bool> opened_ = false;

  PlanarAudioBuffer conversion_buffer_;
  std::vector<const float*> output_buffer_channel_ptrs_;

  ogg_stream_state stream_state_;
  ogg_page ogg_page_;
  ogg_packet ogg_packet_;

  vorbis_info vorbis_info_;
  vorbis_comment vorbis_comment_;
  vorbis_dsp_state vorbis_state_;
  vorbis_block vorbis_block_;
};

}  // namespace guideline::audio

#endif  // PROJECT_GUIDELINE_AUDIO_VORBIS_STREAM_ENCODER_H_
