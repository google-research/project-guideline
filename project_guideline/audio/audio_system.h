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

// Guideline audio system. Generates audio in response to guidance control
// signals to navigate the user.

#ifndef PROJECT_GUIDELINE_AUDIO_AUDIO_SYSTEM_H_
#define PROJECT_GUIDELINE_AUDIO_AUDIO_SYSTEM_H_

#include <atomic>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/notification.h"
#include "absl/time/time.h"
#include "project_guideline/audio/audio_output_stream.h"
#include "project_guideline/audio/sound_pack.h"
#include "project_guideline/audio/sound_player.h"
#include "project_guideline/audio/vorbis_stream_encoder.h"
#include "project_guideline/environment/control_signal.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "resonance_audio_api.h"

namespace guideline::audio {

class AudioSystem {
 public:
  static absl::StatusOr<std::unique_ptr<AudioSystem>> Create(
      const AudioSystemOptions& options,
      std::shared_ptr<logging::GuidelineLogger> logger,
      std::shared_ptr<audio::AudioOutputStream> audio_output_stream);

  ~AudioSystem();

  // Sets the filename to record the audio stream, if audio recording is
  // desired. Must call `EnableRecording` to actually start recording audio
  // data.
  void RecordToFile(absl::string_view recording_filename);

  absl::Status Start();
  absl::Status Stop();

  // Waits until the audio output stream begins.
  absl::Status WaitForAudioStream();

  // Enables the recording of audio data. This can be used to delay the start
  // of the audio recording, e.g. until image frames have started flowing.
  void EnableRecording();

  // Invoked when a new ControlSignal is emitted from the control system.
  void OnControlSignal(const environment::ControlSignal& signal);

  void OnBatteryLevel(int level);

 private:
  explicit AudioSystem(
      std::shared_ptr<vraudio::ResonanceAudioApi> resonance_audio,
      std::unique_ptr<SoundPack> sound_pack,
      std::shared_ptr<audio::AudioOutputStream> audio_output_stream);

  enum class ControlSoundState {
    // The system is waiting for control signals to become available.
    // Transitions to: READY
    INITIALIZING,
    // A control signal is available, playing the "Ready" sound.
    // Transitions to: GUIDING or STOPPING
    READY,
    // Main state to issue guidance signals.
    // Transitions to: STOPPING
    GUIDING,
    // Stop sound is being played.
    // Transitions to: INITIALIZING
    STOPPING
  };

  bool OnMoreData(int16_t* buffer_ptr, size_t num_channel, size_t num_frames);

  absl::Status Initialize();

  void BeginInitializationState() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  void OnExclusiveSoundFinished(SoundId sound_id)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  void StopSoundPlayer();

  const int kLowBatteryWarningThreshold = 25;
  const absl::Duration kLowBatteryWarningInterval = absl::Seconds(30);

  std::shared_ptr<vraudio::ResonanceAudioApi> resonance_audio_;
  std::unique_ptr<SoundPack> sound_pack_;
  std::shared_ptr<audio::AudioOutputStream> audio_output_stream_;
  std::unique_ptr<SoundPlayer> base_sound_player_;
  std::unique_ptr<audio::VorbisStreamEncoder> stream_encoder_ = nullptr;
  std::string recording_filename_ = "";

  absl::Mutex mutex_;
  bool started_ ABSL_GUARDED_BY(mutex_) = false;

  SoundId initializing_sound_;
  SoundId ready_sound_;
  SoundId stop_sound_;
  SoundId low_battery_sound_;

  bool alerting_low_battery_ = false;
  std::atomic<bool> recording_ = false;
  absl::Notification audio_stream_started_;

  ControlSoundState state_ ABSL_GUARDED_BY(mutex_) =
      ControlSoundState::INITIALIZING;
};

}  // namespace guideline::audio

#endif  // PROJECT_GUIDELINE_AUDIO_AUDIO_SYSTEM_H_
