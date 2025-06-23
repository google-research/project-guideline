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

#ifndef PROJECT_GUIDELINE_UTIL_BIQUAD_FILTER_H_
#define PROJECT_GUIDELINE_UTIL_BIQUAD_FILTER_H_

#include "absl/time/time.h"
#include "Eigen/Core"  // keep include

namespace guideline::util {

// Biquad filter coefficients.
struct BiquadFilterCoefficients {
  // Default constructor with identity filter.
  BiquadFilterCoefficients()
      : BiquadFilterCoefficients({1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}) {}

  // Constructor with feedback coefficients coeff_a and feedforward coefficients
  // coeff_b .
  BiquadFilterCoefficients(const Eigen::Vector3f& coeffs_a,
                           const Eigen::Vector3f& coeffs_b)
      : a(coeffs_a), b(coeffs_b) {}

  void Normalize() {
    a /= a[0];
    b /= a[0];
  }

  Eigen::Vector3f a;
  Eigen::Vector3f b;
};

// Streaming biquad filter.
class BiquadFilter {
 public:
  BiquadFilter(const BiquadFilterCoefficients& coeffs);
  float Process(float x);
  void Reset();

 private:
  float b0_;
  Eigen::Vector2f b1b2_;
  Eigen::Vector2f a1a2_;
  Eigen::Matrix2f state_;
};

// Computes biquad filter coefficients for a 2nd order Butterworth low-pass
// filter with the given sample rate and low-pass corner frequency.
BiquadFilterCoefficients ButterworthLowPassFilterCoefficients(
    float sample_rate_hz, float low_pass_corner_frequency_hz);

absl::Duration ComputeBiquadFilterDelay(const BiquadFilterCoefficients& coeffs,
                                        float sample_rate_hz,
                                        float query_frequency_hz);

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_BIQUAD_FILTER_H_
