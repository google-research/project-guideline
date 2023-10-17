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

#include "project_guideline/audio/audio_stream_renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "project_guideline/audio/audio_buffer.h"
#include "project_guideline/audio/resampler.h"
#include "project_guideline/audio/vorbis_stream_decoder.h"
#include "project_guideline/util/status.h"

namespace guideline::audio {

absl::StatusOr<std::unique_ptr<AudioStreamRenderer>>
AudioStreamRenderer::Create(int output_sample_rate_hz, int frames_per_buffer,
                            std::unique_ptr<std::string> ogg_vorbis_data,
                            bool mono_to_stereo) {
  GL_ASSIGN_OR_RETURN(auto decoder,
                      VorbisStreamDecoder::Decode(std::move(ogg_vorbis_data)));

  GL_ASSIGN_OR_RETURN(auto audio_data, decoder->ReadFully());

  if (output_sample_rate_hz != decoder->sample_rate_hz()) {
    Resampler resampler(decoder->sample_rate_hz(), output_sample_rate_hz);
    auto resampled = std::make_unique<PlanarAudioBuffer>();
    resampled->resize(audio_data->size());
    for (int channel = 0; channel < audio_data->size(); channel++) {
      resampler.Resample((*audio_data)[channel], (*resampled)[channel]);
    }
    audio_data = std::move(resampled);
  }

  return absl::WrapUnique(
      new AudioStreamRenderer(output_sample_rate_hz, frames_per_buffer,
                              std::move(audio_data), mono_to_stereo));
}

AudioStreamRenderer::AudioStreamRenderer(
    int output_sample_rate_hz, int frames_per_buffer,
    std::unique_ptr<PlanarAudioBuffer> audio_data, bool mono_to_stereo)
    : output_sample_rate_hz_(output_sample_rate_hz),
      frames_per_buffer_(frames_per_buffer),
      audio_data_(std::move(audio_data)) {
  int num_output_channels = audio_data_->size();

  if (mono_to_stereo && num_output_channels == 1) {
    num_output_channels = 2;
  }

  output_buffer_channel_ptrs_.resize(num_output_channels);
  output_buffer_.resize(num_output_channels);

  for (size_t channel = 0; channel < num_output_channels; ++channel) {
    output_buffer_[channel].resize(static_cast<size_t>(frames_per_buffer));
  }
}

void AudioStreamRenderer::SetLoop(bool loop) {
  absl::MutexLock lock(&mutex_);
  loop_ = loop;

  // If we stop looping while in a loop delay period, just go to end-of-stream
  // immediately. Otherwise allow the sound to finish playing if needed.
  if (remaining_loop_delay_ > absl::ZeroDuration()) {
    remaining_loop_delay_ = absl::ZeroDuration();
    end_of_stream_ = true;
  }
}

void AudioStreamRenderer::SetLoopRepeatDelay(absl::Duration delay) {
  absl::MutexLock lock(&mutex_);
  loop_repeat_delay_ = delay;
}

bool AudioStreamRenderer::IsPlaying() {
  absl::MutexLock lock(&mutex_);
  return playing_ && !end_of_stream_;
}

void AudioStreamRenderer::SetPlaybackRate(float playback_rate) {
  absl::MutexLock lock(&mutex_);
  playback_rate_ = playback_rate;
  static const float kPlaybackRateEpsilon = 0.01f;
  has_playback_rate_ = std::abs(1. - playback_rate_) > kPlaybackRateEpsilon;
}

void AudioStreamRenderer::Play() {
  absl::MutexLock lock(&mutex_);
  playing_ = true;
}

void AudioStreamRenderer::Stop() {
  absl::MutexLock lock(&mutex_);
  playing_ = false;
}

void AudioStreamRenderer::Reset() {
  absl::MutexLock lock(&mutex_);
  data_position_ = 0;
  end_of_stream_ = false;
  playing_ = false;
  current_loop_count_ = 0;
  remaining_loop_delay_ = absl::ZeroDuration();
}

const float** AudioStreamRenderer::GetNextBuffer() {
  absl::MutexLock lock(&mutex_);

  int num_data_frames = (*audio_data_)[0].size();
  int num_channels = GetNumChannels();
  int num_frames_in_output_buffer = 0;

  // If we have a remaining loop delay then fill the output buffer with silence.
  if (loop_ && remaining_loop_delay_ > absl::ZeroDuration()) {
    auto delay_frames =
        std::min(static_cast<int>(absl::ToDoubleSeconds(remaining_loop_delay_) *
                                  output_sample_rate_hz_),
                 frames_per_buffer_);

    for (size_t channel = 0; channel < num_channels; ++channel) {
      std::fill(output_buffer_[channel].begin(),
                output_buffer_[channel].begin() + delay_frames, 0.0f);
    }
    num_frames_in_output_buffer = delay_frames;
    remaining_loop_delay_ -=
        absl::Seconds(num_frames_in_output_buffer /
                      static_cast<float>(output_sample_rate_hz_));
  }

  while (num_frames_in_output_buffer < frames_per_buffer_) {
    const size_t num_frames_available_for_playback =
        num_data_frames - data_position_;
    const size_t num_output_frames_remaining =
        frames_per_buffer_ - num_frames_in_output_buffer;

    size_t num_data_frames_advanced = 0;
    size_t num_frames_copied = 0;
    for (size_t channel = 0; channel < num_channels; ++channel) {
      // Support mono -> stereo
      size_t input_channel = std::min(channel, audio_data_->size() - 1);

      std::vector<float>& input_data = (*audio_data_)[input_channel];
      std::vector<float>& output_data = output_buffer_[channel];

      size_t channel_num_frames_copied = 0;
      size_t channel_num_data_frames_advanced = 0;
      if (has_playback_rate_) {
        // This code dynamically resamples the audio to change the playback
        // rate. This is optimized for frequent calls to SetPlaybackRate().
        // If the playback rate were not changed frequently we could use
        // the Resampler to resample the entire stream upfront.
        channel_num_frames_copied = Resampler::ResampleChunk(
            input_data, output_data, playback_rate_,
            /*start_input_index=*/data_position_,
            /*start_output_index=*/num_frames_in_output_buffer,
            /*max_frames_to_copy=*/num_output_frames_remaining,
            /*num_frames_advanced=*/channel_num_data_frames_advanced);
      } else {
        const size_t num_frames_to_copy = std::min(
            num_frames_available_for_playback, num_output_frames_remaining);
        std::copy_n(&input_data[data_position_], num_frames_to_copy,
                    &(output_data[num_frames_in_output_buffer]));
        channel_num_frames_copied = num_frames_to_copy;
        channel_num_data_frames_advanced = num_frames_to_copy;
      }

      if (channel == 0) {
        num_frames_copied = channel_num_frames_copied;
        num_data_frames_advanced = channel_num_data_frames_advanced;
      } else {
        // Must be the same number of frames copied to each channel.
        CHECK(channel_num_frames_copied == num_frames_copied &&
              channel_num_data_frames_advanced == num_data_frames_advanced);
      }
    }

    num_frames_in_output_buffer += num_frames_copied;
    data_position_ += num_data_frames_advanced;
    data_position_ %= num_data_frames;
    if (data_position_ == 0) {
      if (loop_) {
        ++current_loop_count_;
      } else {
        end_of_stream_ = true;
      }
    }

    if (left_volume_ != 1.) {
      for (int i = 0; i < num_frames_in_output_buffer; i++) {
        output_buffer_[0][i] *= left_volume_;
      }
    }

    if (right_volume_ != 1.) {
      for (int i = 0; i < num_frames_in_output_buffer; i++) {
        output_buffer_[1][i] *= right_volume_;
      }
    }

    if (num_frames_in_output_buffer < frames_per_buffer_) {
      bool zero_fill_remaining = true;

      if (loop_ && num_frames_copied != 0) {
        if (loop_repeat_delay_ > absl::ZeroDuration()) {
          remaining_loop_delay_ = loop_repeat_delay_;
          auto remaining_buffer_duration =
              absl::Seconds((frames_per_buffer_ - num_frames_in_output_buffer) /
                            static_cast<float>(output_sample_rate_hz_));
          // Note we assume the delay is much greater than the duration of
          // a single buffer so delay should not go negative here (but if it
          // did we would just lose a bit precision of the delay time).
          remaining_loop_delay_ -= remaining_buffer_duration;
        } else {
          // For immediate loop we can continue to fill the buffer with the
          // beginning of the sound.
          zero_fill_remaining = false;
        }
      }

      if (zero_fill_remaining) {
        for (size_t channel = 0; channel < num_channels; ++channel) {
          std::fill(
              output_buffer_[channel].begin() + num_frames_in_output_buffer,
              output_buffer_[channel].end(), 0.0f);
        }
        num_frames_in_output_buffer = frames_per_buffer_;
      }
    }
  }

  for (size_t channel = 0; channel < num_channels; ++channel) {
    output_buffer_channel_ptrs_[channel] = &output_buffer_[channel][0];
  }

  return output_buffer_channel_ptrs_.data();
}

size_t AudioStreamRenderer::GetNumChannels() { return output_buffer_.size(); }

int AudioStreamRenderer::GetCurrentLoopCount() {
  absl::MutexLock lock(&mutex_);
  return current_loop_count_;
}

void AudioStreamRenderer::SetStereoVolume(float left_volume,
                                          float right_volume) {
  absl::MutexLock lock(&mutex_);
  CHECK_EQ(output_buffer_.size(), 2);
  left_volume_ = left_volume;
  right_volume_ = right_volume;
}

}  // namespace guideline::audio
