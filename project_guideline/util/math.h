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

#ifndef PROJECT_GUIDELINE_UTIL_MATH_H_
#define PROJECT_GUIDELINE_UTIL_MATH_H_

#include <numeric>

#include "absl/log/check.h"

namespace guideline::util {

template <typename T, typename std::enable_if<std::is_floating_point<T>::value,
                                              T>::type* = nullptr>
inline T Mean(const std::vector<T>& values) {
  CHECK(!values.empty());
  const T sum = std::accumulate(values.begin(), values.end(), 0.);
  return sum / values.size();
}

template <typename T, typename std::enable_if<std::is_floating_point<T>::value,
                                              T>::type* = nullptr>
inline T SumOfSquares(const std::vector<T>& values) {
  if (values.empty()) {
    return 0;
  }
  return std::inner_product(values.begin(), values.end(), values.begin(), 0.);
}

template <typename T, typename std::enable_if<std::is_floating_point<T>::value,
                                              T>::type* = nullptr>
inline T StandardDeviation(const std::vector<T>& values) {
  const T mean = Mean(values);
  const T sum_of_squares = SumOfSquares(values);
  const double mean_of_squares = sum_of_squares / values.size();
  return std::sqrt(mean_of_squares - mean * mean);
}

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_MATH_H_
