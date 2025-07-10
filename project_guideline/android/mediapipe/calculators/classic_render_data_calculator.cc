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

#include <cmath>
#include <memory>

#include "absl/status/status.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/port/ret_check.h"
#include "mediapipe/util/render_data.pb.h"
#include "project_guideline/android/mediapipe/proto/classic_guidance.pb.h"
#include "project_guideline/android/mediapipe/proto/line_detection.pb.h"

namespace guideline {

namespace {

using ::drishti::RenderData;
using ::mediapipe::CalculatorBase;
using ::mediapipe::CalculatorContext;
using ::mediapipe::CalculatorContract;

constexpr char kGuidanceTag[] = "GUIDANCE";
const char kRenderDataTag[] = "RENDER_DATA";

constexpr char kSceneClass[] = "guideline_classic";

constexpr float kDegToRad = M_PI / 180.0f;

}  // namespace

class ClassicRenderDataCalculator : public drishti::CalculatorBase {
 public:
  static absl::Status GetContract(CalculatorContract* cc);
  absl::Status Open(CalculatorContext* cc) override;
  absl::Status Process(CalculatorContext* cc) override;

 private:
};
REGISTER_CALCULATOR(::guideline::ClassicRenderDataCalculator);

absl::Status ClassicRenderDataCalculator::GetContract(CalculatorContract* cc) {
  RET_CHECK(cc->Inputs().HasTag(kGuidanceTag));
  cc->Inputs().Tag(kGuidanceTag).Set<ClassicGuidance>();

  cc->Outputs().Tag(kRenderDataTag).Set<RenderData>();

  return absl::OkStatus();
}

absl::Status ClassicRenderDataCalculator::Open(CalculatorContext* cc) {
  return absl::OkStatus();
}

absl::Status ClassicRenderDataCalculator::Process(CalculatorContext* cc) {
  // Do not produce RenderData for prestream or poststream packets.
  if (cc->InputTimestamp().IsSpecialValue()) {
    return absl::OkStatus();
  }

  auto render_data = std::make_unique<RenderData>();
  render_data->set_scene_class(kSceneClass);

  const auto& guidance = cc->Inputs().Tag(kGuidanceTag).Get<ClassicGuidance>();

  for (const auto& segment : guidance.line_detection().segment()) {
    auto* annotation = render_data->add_render_annotations();
    annotation->set_thickness(4);
    annotation->mutable_color()->set_g(200);
    auto* line = annotation->mutable_line();
    line->set_normalized(true);
    line->set_x_start(segment.bottom().x());
    line->set_y_start(segment.bottom().y());
    line->set_x_end(segment.top().x());
    line->set_y_end(segment.top().y());
  }

  {
    // Draw steering track.
    auto* annotation = render_data->add_render_annotations();
    annotation->set_thickness(4);
    annotation->mutable_color()->set_b(100);
    auto* line = annotation->mutable_line();
    line->set_normalized(true);
    line->set_x_start(0.1);
    line->set_y_start(0.9);
    line->set_x_end(0.9);
    line->set_y_end(0.9);
  }

  if (guidance.stop()) {
    auto* annotation = render_data->add_render_annotations();
    annotation->mutable_color()->set_r(200);
    annotation->set_thickness(2);
    auto* text = annotation->mutable_text();
    text->set_font_height(0.1);
    text->set_normalized(true);
    text->set_display_text("STOP");
    text->set_center_vertically(true);
    text->set_center_horizontally(true);
    text->set_left(0.5);
    text->set_baseline(0.8);
  } else {
    auto* annotation = render_data->add_render_annotations();
    annotation->set_thickness(4);
    annotation->mutable_color()->set_b(200);
    auto* line = annotation->mutable_line();
    float x_pos = guidance.lateral_position() / 0.8 + 0.1;
    line->set_normalized(true);
    line->set_x_start(x_pos);
    line->set_y_start(0.95);
    line->set_x_end(x_pos);
    line->set_y_end(0.85);

    if (guidance.has_detected_curve_angle_degrees()) {
      annotation = render_data->add_render_annotations();
      annotation->set_thickness(4);
      annotation->mutable_color()->set_g(200);
      annotation->mutable_color()->set_b(200);
      auto* arrow = annotation->mutable_arrow();
      arrow->set_normalized(true);
      arrow->set_x_start(x_pos);
      arrow->set_y_start(0.85);
      float curve_angle_degrees = guidance.detected_curve_angle_degrees();
      arrow->set_x_end(x_pos +
                       std::cos(curve_angle_degrees * kDegToRad) * 0.05);
      arrow->set_y_end(x_pos +
                       std::sin(curve_angle_degrees * kDegToRad) * 0.05);
    }
  }

  cc->Outputs()
      .Tag(kRenderDataTag)
      .Add(render_data.release(), cc->InputTimestamp());

  return absl::OkStatus();
}

}  // namespace guideline
