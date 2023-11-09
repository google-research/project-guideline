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

#ifndef PROJECT_GUIDELINE_UTIL_LERP_H_
#define PROJECT_GUIDELINE_UTIL_LERP_H_

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace guideline::util {

template <typename T, typename std::enable_if<std::is_floating_point<T>::value,
                                              int>::type = 0>
static inline T Lerp(T x, T a, T b, T u, T v) {
  return u + ((x - a) / (b - a)) * (v - u);
}

template <typename T, typename std::enable_if<std::is_floating_point<T>::value,
                                              int>::type = 0>
inline T Lerp(T x, T a, T b, T u, T v, T gamma) {
  return u + std::pow((x - a) / (b - a), gamma) * (v - u);
}

template <typename T, typename std::enable_if<std::is_floating_point<T>::value,
                                              int>::type = 0>
inline T ClampedLerp(T x, T a, T b, T u, T v) {
  float high = std::max<T>(a, b);
  float low = std::min<T>(a, b);
  return Lerp(std::min<T>(high, std::max<T>(low, x)), a, b, u, v);
}

template <typename T, typename std::enable_if<std::is_floating_point<T>::value,
                                              int>::type = 0>
inline T ClampedLerp(T x, T a, T b, T u, T v, T gamma) {
  float high = std::max<T>(a, b);
  float low = std::min<T>(a, b);
  return Lerp(std::min<T>(high, std::max<T>(low, x)), a, b, u, v, gamma);
}

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_LERP_H_
