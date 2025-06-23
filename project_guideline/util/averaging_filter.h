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

#ifndef PROJECT_GUIDELINE_UTIL_AVERAGING_FILTER_H_
#define PROJECT_GUIDELINE_UTIL_AVERAGING_FILTER_H_

#include <deque>
#include <type_traits>
#include <utility>

#include "absl/log/check.h"
#include "absl/time/time.h"

namespace guideline::util {

// Filter that averages values over a window of time.
template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>, T>>
class AveragingFilter {
 public:
  explicit AveragingFilter<T>(absl::Duration window) : window_(window) {}

  // Processes a new value and returns the current average over the window,
  // dropping any old values that are now outside the window.
  // This must be called with monotonically increasing timestamps.
  T filter(absl::Time timestamp, T value) {
    values_.emplace_back(timestamp, value);
    sum_ += value;
    while (window_size() > window_) {
      sum_ -= values_.front().second;
      values_.pop_front();
    }
    return average();
  }

  // Returns the current filter average, or 0 if there are no values yet.
  T average() const { return values_.empty() ? 0 : sum_ / values_.size(); }

  // Resets the filter.
  void reset() {
    values_.clear();
    sum_ = 0;
  }

  absl::Duration window_size() const {
    return values_.empty() ? absl::ZeroDuration()
                           : values_.back().first - values_.front().first;
  }

 private:
  absl::Duration window_;
  std::deque<std::pair<absl::Time, T>> values_;
  T sum_ = 0;
};

// Specialization of averaging filter that outputs a boolean value indicating
// whether the average over the window is greater than a threshold. A
// minimum interval can be given so the output value will not switch until at
// least that amount of time has elapsed since the last output change.
template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>, T>>
class LatchingAveragingFilter {
 public:
  explicit LatchingAveragingFilter<T>(T threshold, absl::Duration window,
                                      absl::Duration min_interval,
                                      bool initial_state = false)
      : averaging_filter_(window),
        threshold_(threshold),
        min_interval_(min_interval),
        initial_state_(initial_state),
        last_state_(initial_state) {
    CHECK_GT(window, min_interval);
  }

  bool filter(absl::Time timestamp, T value) {
    bool state = averaging_filter_.filter(timestamp, value) > threshold_;
    if (state != last_state_ &&
        averaging_filter_.window_size() >= min_interval_) {
      last_state_ = state;
      averaging_filter_.reset();
    }
    return last_state_;
  }

  bool state() const { return last_state_; }

  void reset() {
    averaging_filter_.reset();
    last_state_ = initial_state_;
  }

 private:
  AveragingFilter<T> averaging_filter_;
  const T threshold_;
  const absl::Duration min_interval_;
  const bool initial_state_;
  bool last_state_;
};

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_AVERAGING_FILTER_H_
