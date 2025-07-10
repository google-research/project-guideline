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

#include <memory>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/time/time.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/port/ret_check.h"
#include "project_guideline/android/mediapipe/proto/classic_guidance.pb.h"
#include "project_guideline/android/mediapipe/proto/line_detection.pb.h"
#include "project_guideline/util/averaging_filter.h"
#include "project_guideline/util/biquad_filter.h"

namespace guideline {

namespace {
using ::mediapipe::CalculatorBase;
using ::mediapipe::CalculatorContext;
using ::mediapipe::CalculatorContract;

constexpr char kLineDetectionTag[] = "LINE_DETECTION";
constexpr char kGuidanceTag[] = "GUIDANCE";

constexpr float kStopValue = 1.0f;
constexpr float kContinueValue = 0.0f;

}  // namespace

// Creates guidance instructions for the Guideline classic mode.
class ClassicGuidanceCalculator : public CalculatorBase {
 public:
  static absl::Status GetContract(CalculatorContract* cc);

  absl::Status Open(CalculatorContext* cc) override;
  absl::Status Process(CalculatorContext* cc) override;
  absl::Status Close(CalculatorContext* cc) override;

 private:
  std::unique_ptr<util::LatchingAveragingFilter<float>> stop_filter_ = nullptr;
  std::unique_ptr<util::BiquadFilter> lateral_position_filter_ = nullptr;
  std::unique_ptr<util::AveragingFilter<float>> curve_angle_filter_ = nullptr;
};
REGISTER_CALCULATOR(::guideline::ClassicGuidanceCalculator);

absl::Status ClassicGuidanceCalculator::GetContract(CalculatorContract* cc) {
  RET_CHECK(cc->Inputs().HasTag(kLineDetectionTag));
  cc->Inputs().Tag(kLineDetectionTag).Set<LineDetection>();

  RET_CHECK(cc->Outputs().HasTag(kGuidanceTag));
  cc->Outputs().Tag(kGuidanceTag).Set<LineDetection>();

  return absl::OkStatus();
}

absl::Status ClassicGuidanceCalculator::Open(CalculatorContext* cc) {
  stop_filter_ = std::make_unique<util::LatchingAveragingFilter<float>>(
      /*threshold=*/0.8, /*window=*/absl::Seconds(1),
      /*min_interval=*/absl::Seconds(0.5), /*initial_state=*/true);

  // Use a butterworth low-pass filter to help remove the side-to-side noise in
  // the line position incurred while running. This is more accurate and has
  // less latency than using an averaging filter.
  util::BiquadFilterCoefficients butterworth_coeffs =
      util::ButterworthLowPassFilterCoefficients(
          /*sample_rate_hz=*/30,
          /*low_pass_corner_frequency_hz=*/2.0);
  lateral_position_filter_ =
      std::make_unique<util::BiquadFilter>(butterworth_coeffs);

  // Average the curve angle since it should not be impacted by the side-to-side
  // noise.
  curve_angle_filter_ =
      std::make_unique<util::AveragingFilter<float>>(absl::Seconds(1));

  return absl::OkStatus();
}

absl::Status ClassicGuidanceCalculator::Process(CalculatorContext* cc) {
  const auto& line_detection =
      cc->Inputs().Tag(kLineDetectionTag).Get<LineDetection>();
  const absl::Time timestamp =
      absl::FromUnixMicros(cc->InputTimestamp().Microseconds());

  auto guidance = std::make_unique<ClassicGuidance>();
  *guidance->mutable_line_detection() = line_detection;

  bool stop = stop_filter_->filter(
      timestamp, line_detection.has_x_position() ? kContinueValue : kStopValue);
  guidance->set_stop(stop);

  if (line_detection.has_x_position()) {
    float filtered_lateral_position =
        lateral_position_filter_->Process(line_detection.x_position());
    guidance->set_lateral_position(filtered_lateral_position);
  }

  if (line_detection.has_curve_angle_degrees()) {
    float filtered_curve_angle = curve_angle_filter_->filter(
        timestamp, line_detection.curve_angle_degrees());
    guidance->set_detected_curve_angle_degrees(filtered_curve_angle);
  }

  cc->Outputs().Tag(kGuidanceTag).Add(guidance.release(), cc->InputTimestamp());

  return absl::OkStatus();
}

absl::Status ClassicGuidanceCalculator::Close(CalculatorContext* cc) {
  return absl::OkStatus();
}

}  // namespace guideline
