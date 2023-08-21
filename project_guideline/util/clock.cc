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

#include "project_guideline/util/clock.h"

#include "absl/time/clock.h"

namespace guideline::util {

namespace internal {
class SystemClock final : public Clock {
 public:
  absl::Time Now() override { return absl::Now(); }

  void Sleep(absl::Duration d) override { absl::SleepFor(d); }

  void SleepUntil(absl::Time t) override {
    absl::Duration d = t - absl::Now();
    if (d > absl::ZeroDuration()) {
      Sleep(d);
    }
  }
};
}  // namespace internal

std::shared_ptr<Clock> Clock::SystemClock() {
  static std::shared_ptr<internal::SystemClock> system_clock =
      std::make_unique<internal::SystemClock>();
  return system_clock;
}

}  // namespace guideline::util
