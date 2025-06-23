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

#include "project_guideline/util/biquad_filter.h"

#include <cmath>
#include <complex>
#include <limits>

#include "absl/time/time.h"
#include "Eigen/Core"  // keep include

namespace guideline::util {

BiquadFilter::BiquadFilter(const BiquadFilterCoefficients& coeffs) {
  BiquadFilterCoefficients coeffs_normalized = coeffs;
  coeffs_normalized.Normalize();
  b0_ = coeffs_normalized.b[0];
  b1b2_ << coeffs_normalized.b[1], coeffs_normalized.b[2];
  a1a2_ << coeffs_normalized.a[1], coeffs_normalized.a[2];

  state_.setZero();
}

float BiquadFilter::Process(float x) {
  float next_state = x - (state_.row(0) * a1a2_)[0];
  float out = next_state * b0_ + (state_.row(0) * b1b2_)[0];
  state_(0, 1) = state_(0, 0);
  state_(0, 0) = next_state;
  return out;
}

void BiquadFilter::Reset() { state_.setZero(); }

BiquadFilterCoefficients ButterworthLowPassFilterCoefficients(
    float sample_rate_hz, float low_pass_corner_frequency_hz) {
  float rad_s = 2 * sample_rate_hz *
                std::tan(M_PI * low_pass_corner_frequency_hz / sample_rate_hz);
  std::complex<float> pole =
      std::polar<float>(1.0f, 3.0f * M_PI / 4.0f) * rad_s;

  float k = 2 * sample_rate_hz;
  std::complex<float> pole_pair = (k + pole) / (k - pole);
  std::complex<float> conj_pole_pair = std::conj(pole_pair);

  Eigen::Vector3f coeff_a(1, -(pole_pair + conj_pole_pair).real(),
                          (pole_pair * conj_pole_pair).real());
  Eigen::Vector3f coeff_b(1, 2, 1);

  const int kRelativeDegree = 2;
  float b_gain = std::pow(rad_s, kRelativeDegree) /
                 std::abs((pole - k) * (std::conj(pole) - k));
  coeff_b *= b_gain;

  return BiquadFilterCoefficients(coeff_a, coeff_b);
}

absl::Duration ComputeBiquadFilterDelay(const BiquadFilterCoefficients& coeffs,
                                        const float sample_rate_hz,
                                        const float query_frequency_hz) {
  if (std::fabs(query_frequency_hz) < std::numeric_limits<float>::epsilon()) {
    return absl::ZeroDuration();
  }

  const float w = 2 * M_PI * (query_frequency_hz / sample_rate_hz);
  const std::complex<float> z = std::polar<float>(1.0f, -w);

  std::complex<float> numerator(0);
  std::complex<float> denominator(0);
  for (int i = 2; i >= 0; --i) {
    numerator = (z * numerator) + coeffs.b[i];
    denominator = (z * denominator) + coeffs.a[i];
  }

  return absl::Seconds(-std::arg(numerator * std::conj(denominator)) /
                       (2.0f * M_PI * query_frequency_hz));
}

}  // namespace guideline::util
