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

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "absl/random/distributions.h"
#include "absl/random/random.h"
#include "absl/time/time.h"

#define FLOAT_TOLERANCE 1e-3f

namespace guideline::util {
namespace {

TEST(BiquadFilter, IdentityFilter) {
  BiquadFilter filter(BiquadFilterCoefficients{});

  EXPECT_EQ(filter.Process(1.0f), 1.0f);
  EXPECT_EQ(filter.Process(2.0f), 2.0f);
  EXPECT_EQ(filter.Process(3.0f), 3.0f);
  EXPECT_EQ(filter.Process(4.0f), 4.0f);
  EXPECT_EQ(filter.Process(5.0f), 5.0f);
}

TEST(BiquadFilterDelay, IdentityFilter) {
  const float kSampleRateHz = 60.0f;
  const float kQueryFrequencyHz = 10.0f;

  absl::Duration delay = ComputeBiquadFilterDelay(
      BiquadFilterCoefficients{}, kSampleRateHz, kQueryFrequencyHz);

  EXPECT_EQ(delay, absl::ZeroDuration());
}

TEST(BiquadFilterDelay, ButterworthLowPassFilter) {
  const float kSampleRate30Hz = 30.0f;
  const float kSampleRate60Hz = 60.0f;
  const float kLowPassCornerFrequencyHz = 10.0f;

  absl::Duration delay =
      ComputeBiquadFilterDelay(ButterworthLowPassFilterCoefficients(
                                   kSampleRate60Hz, kLowPassCornerFrequencyHz),
                               kSampleRate60Hz, kLowPassCornerFrequencyHz);
  EXPECT_NEAR(absl::ToDoubleMilliseconds(delay), 25, FLOAT_TOLERANCE);

  delay =
      ComputeBiquadFilterDelay(ButterworthLowPassFilterCoefficients(
                                   kSampleRate30Hz, kLowPassCornerFrequencyHz),
                               kSampleRate30Hz, kLowPassCornerFrequencyHz);
  EXPECT_NEAR(absl::ToDoubleMilliseconds(delay), 25, FLOAT_TOLERANCE);

  delay =
      ComputeBiquadFilterDelay(ButterworthLowPassFilterCoefficients(
                                   kSampleRate60Hz, kLowPassCornerFrequencyHz),
                               kSampleRate30Hz, kLowPassCornerFrequencyHz);
  EXPECT_NEAR(absl::ToDoubleMilliseconds(delay), 42.239, FLOAT_TOLERANCE);

  delay = ComputeBiquadFilterDelay(
      ButterworthLowPassFilterCoefficients(kSampleRate60Hz,
                                           kLowPassCornerFrequencyHz),
      kSampleRate60Hz, kLowPassCornerFrequencyHz / 2.0f);
  EXPECT_NEAR(absl::ToDoubleMilliseconds(delay), 22.173, FLOAT_TOLERANCE);

  delay =
      ComputeBiquadFilterDelay(ButterworthLowPassFilterCoefficients(
                                   kSampleRate60Hz, kLowPassCornerFrequencyHz),
                               kSampleRate60Hz, kSampleRate60Hz);
  EXPECT_NEAR(absl::ToDoubleMilliseconds(delay), 0, FLOAT_TOLERANCE);
}

TEST(ButterworthLowPassFilter, Simple) {
  const float kSampleRateHz = 30.0f;
  const float kLowPassCornerFrequencyHz = 4.0f;

  BiquadFilter filter(ButterworthLowPassFilterCoefficients(
      kSampleRateHz, kLowPassCornerFrequencyHz));

  EXPECT_NEAR(filter.Process(1.0f), 0.108f, FLOAT_TOLERANCE);
  EXPECT_NEAR(filter.Process(1.4f), 0.463f, FLOAT_TOLERANCE);
  EXPECT_NEAR(filter.Process(2.8f), 1.088f, FLOAT_TOLERANCE);
  EXPECT_NEAR(filter.Process(1.6f), 1.743f, FLOAT_TOLERANCE);
  EXPECT_NEAR(filter.Process(1.0f), 1.950f, FLOAT_TOLERANCE);
  EXPECT_NEAR(filter.Process(-0.5f), 1.504f, FLOAT_TOLERANCE);
  EXPECT_NEAR(filter.Process(-0.9f), 0.615f, FLOAT_TOLERANCE);
  EXPECT_NEAR(filter.Process(0), -0.177f, FLOAT_TOLERANCE);
  EXPECT_NEAR(filter.Process(0), -0.444f, FLOAT_TOLERANCE);
  EXPECT_NEAR(filter.Process(0), -0.334f, FLOAT_TOLERANCE);
  EXPECT_NEAR(filter.Process(0), -0.155f, FLOAT_TOLERANCE);
  EXPECT_NEAR(filter.Process(0), -0.032f, FLOAT_TOLERANCE);
}

TEST(ButterworthLowPassFilter, NoisySignal) {
  const float kSampleRateHz = 30.0f;
  const float kLowPassCornerFrequencyHz = 2.0f;

  BiquadFilter filter(ButterworthLowPassFilterCoefficients(
      kSampleRateHz, kLowPassCornerFrequencyHz));

  std::seed_seq my_seed_seq({1, 2, 3});
  absl::BitGen bitgen(my_seed_seq);

  std::vector<float> raw_signal;
  std::vector<float> noisy_signal;
  std::vector<float> filtered_signal;

  const int kNumSamples = 1000;
  for (int i = 0; i < kNumSamples; ++i) {
    float signal_value = std::sin((1.0f * i / kNumSamples) * M_PI * 2) * 40;
    raw_signal.push_back(signal_value);

    float noise = std::cos((200.0f * i / kNumSamples) * M_PI * 2) * 80 *
                  absl::Uniform(bitgen, 0.5, 1.0);
    float noisy_signal_value = signal_value + noise;
    noisy_signal.push_back(noisy_signal_value);
    filtered_signal.push_back(filter.Process(noisy_signal_value));
  }

  float noisy_diff_sum = 0.0f;
  float filtered_diff_sum = 0.0f;
  float max_noisy_diff = 0.0f;
  float max_filtered_diff = 0.0f;
  for (int i = 0; i < kNumSamples; ++i) {
    float noisy_diff = std::abs(noisy_signal[i] - raw_signal[i]);
    noisy_diff_sum += noisy_diff;
    max_noisy_diff = std::max(max_noisy_diff, noisy_diff);

    float filtered_diff = std::abs(filtered_signal[i] - raw_signal[i]);
    filtered_diff_sum += filtered_diff;
    max_filtered_diff = std::max(max_filtered_diff, filtered_diff);
  }

  float avg_noisy_diff = noisy_diff_sum / kNumSamples;
  float avg_filtered_diff = filtered_diff_sum / kNumSamples;

  // Check that the average and max differences compared with the raw signal are
  // much less with the filtered signal than the noisy signal.
  EXPECT_GT(avg_noisy_diff / avg_filtered_diff, 8);
  EXPECT_GT(max_noisy_diff / max_filtered_diff, 5);
}

}  // namespace
}  // namespace guideline::util
