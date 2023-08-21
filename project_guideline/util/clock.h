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

#ifndef PROJECT_GUIDELINE_UTIL_CLOCK_H_
#define PROJECT_GUIDELINE_UTIL_CLOCK_H_

#include <memory>

#include "absl/time/time.h"

namespace guideline::util {

// Utility to access functions of the system clock which can be mocked for
// tests.
class Clock {
 public:
  // Gets the real system clock instance.
  static std::shared_ptr<Clock> SystemClock();

  virtual ~Clock() = default;
  virtual absl::Time Now() = 0;
  virtual void Sleep(absl::Duration d) = 0;
  virtual void SleepUntil(absl::Time t) = 0;
};

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_CLOCK_H_
