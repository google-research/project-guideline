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
#include <functional>

namespace guideline::util {

template <typename T, typename std::enable_if<std::is_floating_point<T>::value,
                                              int>::type = 0>
inline T SquareLerpInterpolator(T input) {
  return input * input;
}

template <typename T, typename std::enable_if<std::is_floating_point<T>::value,
                                              int>::type = 0>
static inline T Lerp(T x, T a, T b, T u, T v) {
  return u + ((x - a) / (b - a)) * (v - u);
}

template <typename T, typename std::enable_if<std::is_floating_point<T>::value,
                                              int>::type = 0>
inline T Lerp(T x, T a, T b, T u, T v, T (*interpolator)(T)) {
  return u + interpolator((x - a) / (b - a)) * (v - u);
}

template <typename T, typename std::enable_if<std::is_floating_point<T>::value,
                                              int>::type = 0>
inline T ClampedLerp(T x, T a, T b, T u, T v) {
  return Lerp(std::min(b, std::max(a, x)), a, b, u, v);
}

template <typename T, typename std::enable_if<std::is_floating_point<T>::value,
                                              int>::type = 0>
inline T ClampedLerp(T x, T a, T b, T u, T v, T (*interpolator)(T)) {
  return Lerp(std::min(b, std::max(a, x)), a, b, u, v, interpolator);
}

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_LERP_H_
