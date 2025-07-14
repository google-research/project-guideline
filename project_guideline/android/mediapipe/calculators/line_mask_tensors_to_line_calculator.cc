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

#include <array>
#include <memory>
#include <optional>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "Eigen/Core"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image.h"
#include "mediapipe/framework/formats/tensor.h"
#include "mediapipe/framework/port/ret_check.h"
#include "project_guideline/android/mediapipe/proto/line_detection.pb.h"
#include "project_guideline/util/hough_transform.h"
#include "project_guideline/util/lerp.h"

namespace guideline {

namespace {
using ::mediapipe::CalculatorBase;
using ::mediapipe::CalculatorContext;
using ::mediapipe::CalculatorContract;
using ::mediapipe::Image;
using ::mediapipe::Tensor;

constexpr char kImageTag[] = "IMAGE";
constexpr char kTensorsTag[] = "LINE_MASK_TENSORS";
constexpr char kLineDetectionTag[] = "LINE_DETECTION";
constexpr char kLetterboxPaddingTag[] = "LETTERBOX_PADDING";

const int kMaskWidth = 65;
const int kMaskHeight = 65;

inline std::optional<util::HoughTransformResult> ValidateResult(
    const util::HoughTransformResult& result, float score_threshold) {
  if (result.score < score_threshold) {
    return std::nullopt;
  }
  return result;
}

Eigen::Vector2f Interpolate(const Eigen::Vector2f& a, const Eigen::Vector2f& b,
                            float t) {
  return b * t + a * (1 - t);
}

Eigen::Vector2f ComputeProjectedIntercept(
    const util::HoughTransformResult bottom_result,
    const std::optional<util::HoughTransformResult> middle_result,
    const std::optional<util::HoughTransformResult> top_result,
    float projection_amount) {
  if (projection_amount <= 0.5) {
    if (!middle_result.has_value()) {
      return bottom_result.bottom_intercept;
    }
    return Interpolate(bottom_result.bottom_intercept,
                       middle_result->bottom_intercept,
                       projection_amount / 0.5);
  } else {
    if (!middle_result.has_value() && !top_result.has_value()) {
      return bottom_result.bottom_intercept;
    } else if (!top_result.has_value()) {
      return middle_result->bottom_intercept;
    } else if (!middle_result.has_value()) {
      return top_result->bottom_intercept;
    }
    return Interpolate(middle_result->bottom_intercept,
                       top_result->bottom_intercept,
                       (projection_amount - 0.5) / 0.5);
  }
}

void AddLineSegment(const std::optional<util::HoughTransformResult> result,
                    LineDetection& detection) {
  if (!result.has_value()) {
    return;
  }
  auto* segment = detection.add_segment();
  auto* bottom = segment->mutable_bottom();
  bottom->set_x(result->bottom_intercept.x());
  bottom->set_y(result->bottom_intercept.y());
  auto* top = segment->mutable_top();
  top->set_x(result->top_intercept.x());
  top->set_y(result->top_intercept.y());
}

}  // namespace

// Converts Guideline model output tensors to LineDetection proto.
class LineMaskTensorsToLineCalculator : public CalculatorBase {
 public:
  static absl::Status GetContract(CalculatorContract* cc);

  absl::Status Open(CalculatorContext* cc) override;
  absl::Status Process(CalculatorContext* cc) override;
  absl::Status Close(CalculatorContext* cc) override;

 private:
  bool enable_curve_detection_;
  std::unique_ptr<util::HoughTransform> hough_transform_;
};
REGISTER_CALCULATOR(::guideline::LineMaskTensorsToLineCalculator);

absl::Status LineMaskTensorsToLineCalculator::GetContract(
    CalculatorContract* cc) {
  RET_CHECK(cc->Inputs().HasTag(kTensorsTag));
  cc->Inputs().Tag(kTensorsTag).Set<std::vector<Tensor>>();

  RET_CHECK(cc->Inputs().HasTag(kImageTag));
  cc->Inputs().Tag(kImageTag).Set<Image>();

  if (cc->Inputs().HasTag(kLetterboxPaddingTag)) {
    cc->Inputs().Tag(kLetterboxPaddingTag).Set<std::array<float, 4>>();
  }

  RET_CHECK(cc->Outputs().HasTag(kLineDetectionTag));
  cc->Outputs().Tag(kLineDetectionTag).Set<LineDetection>();

  return absl::OkStatus();
}

absl::Status LineMaskTensorsToLineCalculator::Open(CalculatorContext* cc) {
  enable_curve_detection_ = false;
  hough_transform_ = std::make_unique<util::HoughTransform>(
      /* position_steps= */ kMaskWidth, /*angle_steps=*/180,
      /*confidence_threshold=*/0.2, /*mask_width=*/kMaskWidth,
      /*mask_height=*/kMaskHeight,
      /*num_segments=*/enable_curve_detection_ ? 3 : 1,
      /*segment_overlap_pct=*/0.1);
  return absl::OkStatus();
}

absl::Status LineMaskTensorsToLineCalculator::Process(CalculatorContext* cc) {
  const auto& image = cc->Inputs().Tag(kImageTag).Get<Image>();

  const auto& input_tensors =
      cc->Inputs().Tag(kTensorsTag).Get<std::vector<Tensor>>();

  std::optional<std::array<float, 4>> letterbox_padding = std::nullopt;
  if (cc->Inputs().HasTag(kLetterboxPaddingTag)) {
    letterbox_padding =
        cc->Inputs().Tag(kLetterboxPaddingTag).Get<std::array<float, 4>>();
  }

  CHECK_EQ(input_tensors.size(), 1);
  const Tensor& single_tensor = input_tensors[0];

  CHECK_EQ(3, single_tensor.shape().dims.size());
  CHECK_EQ(1, single_tensor.shape().dims[0]);
  CHECK_EQ(kMaskWidth, single_tensor.shape().dims[1]);
  CHECK_EQ(kMaskHeight, single_tensor.shape().dims[2]);

  const float* buffer = single_tensor.GetCpuReadView().buffer<float>();
  cv::Mat mask_mat(kMaskHeight, kMaskWidth, CV_32F, const_cast<float*>(buffer));

  std::vector<util::HoughTransformResult> results =
      hough_transform_->Process(mask_mat);

  const float kLineScoreThreshold = 2.0f;
  CHECK_EQ(results.size(), enable_curve_detection_ ? 3 : 1);

  std::optional<util::HoughTransformResult> bottom_result = std::nullopt;
  std::optional<util::HoughTransformResult> middle_result = std::nullopt;
  std::optional<util::HoughTransformResult> top_result = std::nullopt;

  if (enable_curve_detection_) {
    bottom_result = ValidateResult(results[2], kLineScoreThreshold);
    middle_result = ValidateResult(results[1], kLineScoreThreshold);
    top_result = ValidateResult(results[0], kLineScoreThreshold);

    if (!top_result.has_value() && middle_result.has_value()) {
      top_result = middle_result;
    }
  } else {
    bottom_result = ValidateResult(results[0], kLineScoreThreshold);
  }

  auto detection = std::make_unique<LineDetection>();

  if (bottom_result.has_value()) {
    AddLineSegment(bottom_result, *detection);
    AddLineSegment(middle_result, *detection);
    AddLineSegment(top_result, *detection);

    const float kProjectionAmount = 0.3;
    Eigen::Vector2f projected_intercept = ComputeProjectedIntercept(
        *bottom_result, middle_result, top_result, kProjectionAmount);
    float position =
        util::ClampedLerp<float>(projected_intercept.x(), 0, kMaskWidth, -1, 1);
    detection->set_x_position(position);

    if (top_result.has_value()) {
      float bottom_angle =
          bottom_result->GetAngleDegrees(image.width(), image.height());
      float top_angle =
          top_result->GetAngleDegrees(image.width(), image.height());
      float angle_diff = top_angle - bottom_angle;
      detection->set_curve_angle_degrees(angle_diff);
    }
  }

  cc->Outputs()
      .Tag(kLineDetectionTag)
      .Add(detection.release(), cc->InputTimestamp());

  return absl::OkStatus();
}

absl::Status LineMaskTensorsToLineCalculator::Close(CalculatorContext* cc) {
  return absl::OkStatus();
}

}  // namespace guideline
