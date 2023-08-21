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

#ifndef PROJECT_GUIDELINE_TESTING_SIMULATED_CLOCK_H_
#define PROJECT_GUIDELINE_TESTING_SIMULATED_CLOCK_H_

#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "project_guideline/util/clock.h"

namespace guideline::testing {

// Implementation of Clock that can be used for testing.
class SimulatedClock : public util::Clock {
 public:
  ~SimulatedClock() override { SetTime(absl::InfiniteFuture()); }

  void SetTime(absl::Time t) {
    absl::WriterMutexLock lock(&mutex_);
    now_ = t;
  }

  void AdvanceTime(absl::Duration d) {
    absl::WriterMutexLock lock(&mutex_);
    now_ += d;
  }

  absl::Time Now() override {
    absl::ReaderMutexLock lock(&mutex_);
    return now_;
  }

  void Sleep(absl::Duration d) override { SleepUntil(Now() + d); }

  void SleepUntil(absl::Time t) override {
    absl::ReaderMutexLock lock(&mutex_);
    auto sleep_done = [this, t]() ABSL_SHARED_LOCKS_REQUIRED(mutex_) {
      return now_ >= t;
    };
    mutex_.Await(absl::Condition(&sleep_done));
  }

 private:
  absl::Mutex mutex_;
  absl::Time now_ ABSL_GUARDED_BY(mutex_);
};

}  // namespace guideline::testing

#endif  // PROJECT_GUIDELINE_TESTING_SIMULATED_CLOCK_H_
