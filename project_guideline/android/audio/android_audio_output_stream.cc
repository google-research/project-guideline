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


#include "project_guideline/android/audio/android_audio_output_stream.h"

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"

namespace guideline::audio {

namespace {
constexpr size_t kNumPlaybackBuffers = 2;
}  // namespace

absl::StatusOr<std::unique_ptr<AndroidAudioOutputStream>>
AndroidAudioOutputStream::Create(size_t frames_per_buffer, int sample_rate_hz) {
  AAudioStreamBuilder *builder;
  aaudio_result_t result = AAudio_createStreamBuilder(&builder);
  if (result != AAUDIO_OK) {
    return absl::InternalError(
        absl::StrCat("Failed to create AAudio StreamBuilder"));
  }
  AAudioStreamBuilder_setDirection(builder, AAUDIO_DIRECTION_OUTPUT);
  AAudioStreamBuilder_setPerformanceMode(builder,
                                         AAUDIO_PERFORMANCE_MODE_LOW_LATENCY);
  AAudioStreamBuilder_setFormat(builder, AAUDIO_FORMAT_PCM_I16);
  AAudioStreamBuilder_setSamplesPerFrame(builder, kDefaultNumChannels);
  AAudioStreamBuilder_setDirection(builder, AAUDIO_DIRECTION_OUTPUT);
  AAudioStreamBuilder_setSampleRate(builder, sample_rate_hz);
  AAudioStreamBuilder_setBufferCapacityInFrames(
      builder, frames_per_buffer * kNumPlaybackBuffers);
  AAudioStreamBuilder_setFramesPerDataCallback(builder, frames_per_buffer);

  auto stream = absl::WrapUnique(new AndroidAudioOutputStream(
      kDefaultNumChannels, frames_per_buffer, sample_rate_hz, builder));

  AAudioStreamBuilder_setDataCallback(builder, DataCallback, stream.get());
  AAudioStreamBuilder_setErrorCallback(builder, ErrorCallback, stream.get());

  return stream;
}

AndroidAudioOutputStream::AndroidAudioOutputStream(
    size_t num_channels, size_t frames_per_buffer, int sample_rate_hz,
    AAudioStreamBuilder *stream_builder)
    : AudioOutputStream(sample_rate_hz, frames_per_buffer, num_channels),
      stream_builder_(stream_builder) {}

AndroidAudioOutputStream::~AndroidAudioOutputStream() {
  if (aaudio_stream_) {
    auto status = StopPlayback();
    if (!status.ok()) {
      LOG(ERROR) << "Failed to stop playback: " << status;
    }
  }
  {
    absl::MutexLock lock(&restart_thread_mutex_);
    if (restart_thread_) {
      restart_thread_->join();
      restart_thread_ = nullptr;
    }
  }
  if (AAudioStreamBuilder_delete(stream_builder_) != AAUDIO_OK) {
    LOG(ERROR) << "Failed to delete AAudioStreamBuilder";
  }
}

absl::Status AndroidAudioOutputStream::StartPlayback(
    const AudioOutputStream::Callback &callback) {
  absl::MutexLock lock(&run_mutex_);
  {
    absl::MutexLock lock(&callback_mutex_);
    CHECK(!callback_) << "Playback already started";
    callback_ = callback;
  }

  return StartStream();
}

absl::Status AndroidAudioOutputStream::StartStream() {
  if (AAudioStreamBuilder_openStream(stream_builder_, &aaudio_stream_) !=
      AAUDIO_OK) {
    return absl::InternalError("Failed to open stream");
  }
  if (AAudioStream_getFormat(aaudio_stream_) != AAUDIO_FORMAT_PCM_I16) {
    return absl::InternalError("Failed to set PCM_I16 format");
  }
  if (AAudioStream_getSampleRate(aaudio_stream_) != GetSampleRateHz()) {
    return absl::InternalError("Failed to set sample rate");
  }
  if (static_cast<size_t>(AAudioStream_getChannelCount(aaudio_stream_)) !=
      GetNumChannels()) {
    return absl::InternalError("Failed to set channel count");
  }
  if (static_cast<size_t>(AAudioStream_getPerformanceMode(aaudio_stream_)) !=
      AAUDIO_PERFORMANCE_MODE_LOW_LATENCY) {
    return absl::InternalError("Failed to set low-latency mode");
  }
  if (AAudioStream_requestStart(aaudio_stream_) != AAUDIO_OK) {
    return absl::InternalError("Failed to start stream");
  }

  return absl::OkStatus();
}

absl::Status AndroidAudioOutputStream::StopPlayback() {
  absl::MutexLock lock(&run_mutex_);
  {
    absl::MutexLock lock(&callback_mutex_);
    if (callback_ == nullptr) {
      // Already stopped.
      return absl::OkStatus();
    }
    callback_ = nullptr;
  }
  return StopStream();
}

absl::Status AndroidAudioOutputStream::StopStream() {
  CHECK(aaudio_stream_) << "Stream already stopped";

  if (AAudioStream_requestStop(aaudio_stream_) != AAUDIO_OK) {
    return absl::InternalError("Failed to stop stream");
  }

  aaudio_stream_state_t current_state = AAudioStream_getState(aaudio_stream_);
  aaudio_result_t result = AAUDIO_OK;
  constexpr int64_t kTimeoutNanoseconds = 1e8;
  while (result == AAUDIO_OK && current_state != AAUDIO_STREAM_STATE_STOPPED) {
    result = AAudioStream_waitForStateChange(
        aaudio_stream_, current_state, &current_state, kTimeoutNanoseconds);
  }
  if (result != AAUDIO_OK) {
    return absl::InternalError("Failed to stop stream");
  }

  if (AAudioStream_close(aaudio_stream_) != AAUDIO_OK) {
    return absl::InternalError("Failed to close stream");
  }
  aaudio_stream_ = nullptr;
  return absl::OkStatus();
}

void AndroidAudioOutputStream::RestartStream() {
  {
    absl::MutexLock lock(&run_mutex_);
    {
      absl::MutexLock lock(&callback_mutex_);
      if (callback_) {
        return;
      }
    }

    auto status = StopStream();
    if (!status.ok()) {
      LOG(WARNING) << "Failed to stop stream: " << status;
    }
    status = StartStream();
    if (!status.ok()) {
      LOG(ERROR) << "Failed to start stream: " << status;
    }
  }
  absl::MutexLock lock(&restart_thread_mutex_);
  restart_thread_ = nullptr;
}

aaudio_data_callback_result_t AndroidAudioOutputStream::DataCallback(
    AAudioStream *stream, void *user_data, void *audio_data,
    int32_t num_frames) {
  return reinterpret_cast<AndroidAudioOutputStream *>(user_data)->OnData(
      stream, audio_data, num_frames);
}

void AndroidAudioOutputStream::ErrorCallback(AAudioStream *stream,
                                             void *user_data,
                                             aaudio_result_t error) {
  reinterpret_cast<AndroidAudioOutputStream *>(user_data)->OnError(stream,
                                                                   error);
}

aaudio_data_callback_result_t AndroidAudioOutputStream::OnData(
    AAudioStream *stream, void *audio_data, int32_t num_frames) {
  int16_t *const audio_data_int16 = reinterpret_cast<int16_t *>(audio_data);
  AudioOutputStream::Callback callback;
  {
    absl::MutexLock lock(&callback_mutex_);
    callback = callback_;
  }

  if (callback && callback(audio_data_int16, GetNumChannels(), num_frames)) {
    return AAUDIO_CALLBACK_RESULT_CONTINUE;
  }
  // Fill the buffer with silence if no data from callback.
  std::fill_n(audio_data_int16, num_frames, 0);
  return callback ? AAUDIO_CALLBACK_RESULT_CONTINUE
                  : AAUDIO_CALLBACK_RESULT_STOP;
}

void AndroidAudioOutputStream::OnError(AAudioStream *stream,
                                       aaudio_result_t error) {
  switch (error) {
    case AAUDIO_ERROR_DISCONNECTED:
      LOG(WARNING) << "AAudio disconnected, restarting...";
      {
        absl::MutexLock lock(&restart_thread_mutex_);
        if (restart_thread_ == nullptr) {
          restart_thread_ =
              std::make_unique<std::thread>([&] { RestartStream(); });
        }
      }
      break;
    default:
      LOG(ERROR) << "AAudio error: " << error;
      break;
  }
}

}  // namespace guideline::audio
