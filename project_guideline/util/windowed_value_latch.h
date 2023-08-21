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

// Utility that averages values over a specified window and toggles the boolean
// output value when the average crosses a threshold. Once the value changes,
// the window is cleared and the process repeats. A minimum number of samples is
// required (min_interval) before any output change. This can be used to filter
// an unstable input value.
//
// A boolean input could be represented as values 0.0, 1.0 and threshold=0.5.

#ifndef PROJECT_GUIDELINE_UTIL_WINDOWED_VALUE_LATCH_H_
#define PROJECT_GUIDELINE_UTIL_WINDOWED_VALUE_LATCH_H_

#include <deque>
#include <numeric>
#include <type_traits>

#include "absl/log/check.h"
#include "absl/synchronization/mutex.h"

namespace guideline::util {

template <typename T>
class WindowedValueLatch {
  static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type.");

 public:
  WindowedValueLatch(T value_threshold, uint16_t window_size,
                     uint16_t min_interval, bool initial_state = false)
      : value_threshold_(value_threshold),
        window_size_(window_size),
        min_interval_(min_interval),
        initial_state_(initial_state),
        last_state_(initial_state) {
    CHECK_LE(min_interval, window_size) << "min_interval must be < window_size";
  }

  bool Update(T value) {
    absl::MutexLock lock(&mutex_);
    values_.push_back(value);
    if (values_.size() >= window_size_) {
      values_.pop_front();
    }
    if (values_.size() >= min_interval_) {
      T avg = std::accumulate(values_.begin(), values_.end(), T(0)) /
              values_.size();
      bool state = avg > value_threshold_;
      if (state != last_state_) {
        last_state_ = state;
        values_.clear();
      }
    }
    return last_state_;
  }

  void Reset() {
    absl::MutexLock lock(&mutex_);
    values_.clear();
    last_state_ = initial_state_;
  }

 private:
  absl::Mutex mutex_;
  const T value_threshold_;
  const uint16_t window_size_;
  const uint16_t min_interval_;
  const bool initial_state_;
  std::deque<T> values_;
  bool last_state_;
};

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_WINDOWED_VALUE_LATCH_H_
