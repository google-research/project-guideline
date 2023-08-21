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

#include "project_guideline/audio/sink_audio_output_stream.h"

#include <memory>

#include "gtest/gtest.h"
#include "absl/functional/bind_front.h"
#include "absl/log/check.h"
#include "absl/synchronization/notification.h"
#include "absl/time/time.h"
#include "project_guideline/testing/simulated_clock.h"
#include "project_guideline/testing/status_matchers.h"

namespace guideline::audio {
namespace {

class FakeAudioCallback {
 public:
  bool OnMoreData(int16_t* buffer_ptr, size_t num_channel, size_t num_frames) {
    absl::MutexLock lock(&mutex_);
    ++num_data_callbacks_;
    if (next_callback_notification_ &&
        !next_callback_notification_->HasBeenNotified()) {
      next_callback_notification_->Notify();
    }
    return true;
  }

  bool WaitForCallback() {
    absl::Notification* notification;
    {
      absl::MutexLock lock(&mutex_);
      notification = next_callback_notification_.get();
    }
    notification->WaitForNotificationWithTimeout(absl::Seconds(1));
    bool had_callback = notification->HasBeenNotified();
    {
      absl::MutexLock lock(&mutex_);
      next_callback_notification_ = std::make_unique<absl::Notification>();
    }
    return had_callback;
  }

  int GetNumDataCallbacks() {
    absl::MutexLock lock(&mutex_);
    return num_data_callbacks_;
  }

 private:
  std::unique_ptr<absl::Notification> next_callback_notification_
      ABSL_GUARDED_BY(mutex_) = std::make_unique<absl::Notification>();
  int num_data_callbacks_ ABSL_GUARDED_BY(mutex_) = 0;
  absl::Mutex mutex_;
};

class SinkAudioOutputStreamTest : public ::testing::Test {
 public:
  void SetUp() override {
    clock_ = std::make_shared<testing::SimulatedClock>();
    audio_output_ = std::make_unique<SinkAudioOutputStream>(clock_);
    callback_ = std::make_unique<FakeAudioCallback>();
  }

  void TearDown() override {
    clock_->AdvanceTime(absl::Seconds(30));
    CHECK_OK(audio_output_->StopPlayback());
  }

  std::shared_ptr<testing::SimulatedClock> clock_;
  std::unique_ptr<SinkAudioOutputStream> audio_output_;
  std::unique_ptr<FakeAudioCallback> callback_;
};

TEST_F(SinkAudioOutputStreamTest, MoreDataCallbacks) {
  ASSERT_TRUE(audio_output_
                  ->StartPlayback(absl::bind_front(
                      &FakeAudioCallback::OnMoreData, callback_.get()))
                  .ok());

  ASSERT_TRUE(callback_->WaitForCallback());
  ASSERT_EQ(1, callback_->GetNumDataCallbacks());

  // Advance time by less than 1 frame and ensure there is no additional data
  // callback yet.
  clock_->AdvanceTime(absl::Nanoseconds(5));
  absl::SleepFor(absl::Milliseconds(100));
  ASSERT_EQ(1, callback_->GetNumDataCallbacks());

  clock_->AdvanceTime(
      absl::Seconds(1.0 / (audio_output_->GetSampleRateHz() /
                           audio_output_->GetFramesPerBuffer())));
  ASSERT_TRUE(callback_->WaitForCallback());
  ASSERT_EQ(2, callback_->GetNumDataCallbacks());
}

TEST_F(SinkAudioOutputStreamTest, BufferUnderrun) {
  ASSERT_TRUE(audio_output_
                  ->StartPlayback(absl::bind_front(
                      &FakeAudioCallback::OnMoreData, callback_.get()))
                  .ok());

  ASSERT_TRUE(callback_->WaitForCallback());
  ASSERT_EQ(1, callback_->GetNumDataCallbacks());

  clock_->AdvanceTime(absl::Seconds(1));
  ASSERT_TRUE(callback_->WaitForCallback());
  ASSERT_GT(callback_->GetNumDataCallbacks(), 1);

  clock_->AdvanceTime(absl::Seconds(1));
  ASSERT_TRUE(callback_->WaitForCallback());

  ASSERT_GE(audio_output_->GetNumBufferUnderruns(), 2);
}

}  // namespace
}  // namespace guideline::audio
