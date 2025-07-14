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
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/tensor.h"
#include "mediapipe/framework/port/ret_check.h"
#include "mediapipe/gpu/gl_calculator_helper.h"
#include "mediapipe/gpu/gl_simple_shaders.h"
#include "mediapipe/gpu/gpu_buffer.h"
#include "mediapipe/gpu/shader_util.h"
#include "project_guideline/android/mediapipe/proto/line_detection.pb.h"
#include "project_guideline/util/status.h"
#include "project_guideline/visualization/gl_util.h"

namespace guideline {

namespace {

using ::mediapipe::CalculatorBase;
using ::mediapipe::CalculatorContext;
using ::mediapipe::CalculatorContract;
using ::mediapipe::GlCalculatorHelper;
using ::mediapipe::GpuBuffer;
using ::mediapipe::Tensor;
using visualization::CheckGlError;

constexpr char kImageGpuTag[] = "IMAGE_GPU";
constexpr char kMaskTensorsTag[] = "MASK_TENSORS";

const GLchar* const kMaskFragmentShader = R"(
    DEFAULT_PRECISION(highp, float)
    in vec2 sample_coordinate;
    uniform sampler2D input_frame;
    uniform sampler2D mask_texture;

    vec4 blend(vec4 under, vec4 over) {
      vec4 result = mix(under, over, over.a);
      result.a = over.a + under.a * (1.0 - over.a);
      return result;
    }

    void main() {
      vec3 inputColor = texture2D(input_frame, sample_coordinate).rgb;
      // Render the line segmentation mask as green, with alpha based on
      // the confidence value.
      float maskTexel = float(texture2D(mask_texture, sample_coordinate));
      vec4 maskColor = vec4(0., 1., 0., maskTexel * 0.6);
      gl_FragColor = blend(vec4(inputColor, 1.), maskColor);
    }
)";

enum { ATTRIB_VERTEX, ATTRIB_TEXTURE_POSITION, NUM_ATTRIBUTES };

}  // namespace

class MaskOverlayCalculator : public CalculatorBase {
 public:
  static absl::Status GetContract(CalculatorContract* cc);
  absl::Status Open(CalculatorContext* cc) override;
  absl::Status Process(CalculatorContext* cc) override;
  absl::Status Close(CalculatorContext* cc) override;

 private:
  absl::Status GlSetup();

  GlCalculatorHelper gl_helper_;
  bool initialized_ = false;
  GLuint program_ = 0;
  GLuint mask_texture_ = 0;
};
REGISTER_CALCULATOR(::guideline::MaskOverlayCalculator);

absl::Status MaskOverlayCalculator::GetContract(CalculatorContract* cc) {
  GL_RETURN_IF_ERROR(GlCalculatorHelper::UpdateContract(cc));

  RET_CHECK(cc->Inputs().HasTag(kMaskTensorsTag));
  cc->Inputs().Tag(kMaskTensorsTag).Set<std::vector<Tensor>>();

  RET_CHECK(cc->Inputs().HasTag(kImageGpuTag));
  cc->Inputs().Tag(kImageGpuTag).Set<GpuBuffer>();
  RET_CHECK(cc->Outputs().HasTag(kImageGpuTag));
  cc->Inputs().Tag(kMaskTensorsTag).Set<std::vector<Tensor>>();

  cc->Outputs().Tag(kImageGpuTag).Set<GpuBuffer>();

  return absl::OkStatus();
}

absl::Status MaskOverlayCalculator::Open(CalculatorContext* cc) {
  cc->SetOffset(mediapipe::TimestampDiff(0));
  return gl_helper_.Open(cc);
}

absl::Status MaskOverlayCalculator::GlSetup() {
  const GLint attr_location[NUM_ATTRIBUTES] = {
      ATTRIB_VERTEX,
      ATTRIB_TEXTURE_POSITION,
  };
  const GLchar* attr_name[NUM_ATTRIBUTES] = {
      "position",
      "texture_coordinate",
  };

  const std::string shader_src = absl::StrCat(
      drishti::kDrishtiFragmentShaderPreamble, kMaskFragmentShader);
  drishti::GlhCreateProgram(drishti::kBasicVertexShader, shader_src.c_str(),
                            NUM_ATTRIBUTES, (const GLchar**)&attr_name[0],
                            attr_location, &program_);
  RET_CHECK(program_) << "Failed to create program";

  glUseProgram(program_);
  CheckGlError("use program");

  glUniform1i(glGetUniformLocation(program_, "input_frame"), 1);
  glUniform1i(glGetUniformLocation(program_, "mask_texture"), 2);
  CheckGlError("get uniform locations");

  glGenTextures(1, &mask_texture_);
  glBindTexture(GL_TEXTURE_2D, mask_texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_2D, 0);
  CheckGlError("bind mask texture");

  return absl::OkStatus();
}

absl::Status MaskOverlayCalculator::Process(CalculatorContext* cc) {
  return gl_helper_.RunInGlContext([this, cc]() -> absl::Status {
    if (!initialized_) {
      GL_RETURN_IF_ERROR(GlSetup());
      initialized_ = true;
    }

    const auto& input_tensors =
        cc->Inputs().Tag(kMaskTensorsTag).Get<std::vector<Tensor>>();

    const auto& input_buffer =
        cc->Inputs().Tag(kImageGpuTag).Get<drishti::GpuBuffer>();
    auto input_texture = gl_helper_.CreateSourceTexture(input_buffer);
    auto output_texture = gl_helper_.CreateDestinationTexture(
        input_buffer.width(), input_buffer.height());

    // Upload mask texture to GPU.
    {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      auto& tensor = input_tensors[0];
      const int mask_width = tensor.shape().dims[1];
      const int mask_height = tensor.shape().dims[2];

      glBindTexture(GL_TEXTURE_2D, mask_texture_);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, mask_width, mask_height, 0,
                   GL_RED, GL_FLOAT, tensor.GetCpuReadView().buffer<float>());
      glBindTexture(GL_TEXTURE_2D, 0);
      CheckGlError("update mask texture");
    }

    // Bind textures.
    {
      gl_helper_.BindFramebuffer(output_texture);
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, input_texture.name());
      glActiveTexture(GL_TEXTURE2);
      glBindTexture(GL_TEXTURE_2D, mask_texture_);
    }

    GLuint vbo[2];
    GLuint vao;

    // Draw.
    {
      glUseProgram(program_);
      glGenBuffers(2, vbo);
      glGenVertexArrays(1, &vao);
      glBindVertexArray(vao);

      // vbo0
      glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
      glBufferData(GL_ARRAY_BUFFER, 4 * 2 * sizeof(GLfloat),
                   drishti::kBasicSquareVertices, GL_STATIC_DRAW);
      glEnableVertexAttribArray(ATTRIB_VERTEX);
      glVertexAttribPointer(ATTRIB_VERTEX, 2, GL_FLOAT, 0, 0, nullptr);

      // vbo1
      glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
      glBufferData(GL_ARRAY_BUFFER, 4 * 2 * sizeof(GLfloat),
                   drishti::kBasicTextureVertices, GL_STATIC_DRAW);
      glEnableVertexAttribArray(ATTRIB_TEXTURE_POSITION);
      glVertexAttribPointer(ATTRIB_TEXTURE_POSITION, 2, GL_FLOAT, 0, 0,
                            nullptr);

      glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    }

    // Cleanup.
    {
      glDisableVertexAttribArray(ATTRIB_VERTEX);
      glDisableVertexAttribArray(ATTRIB_TEXTURE_POSITION);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      glBindVertexArray(0);
      glDeleteVertexArrays(1, &vao);
      glDeleteBuffers(2, vbo);

      glActiveTexture(GL_TEXTURE2);
      glBindTexture(GL_TEXTURE_2D, 0);
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, 0);
      glFlush();
    }

    glFlush();

    auto output_buffer = output_texture.GetFrame<mediapipe::GpuBuffer>();
    cc->Outputs()
        .Tag(kImageGpuTag)
        .Add(output_buffer.release(), cc->InputTimestamp());

    input_texture.Release();
    output_texture.Release();

    return absl::OkStatus();
  });
}

absl::Status MaskOverlayCalculator::Close(CalculatorContext* cc) {
  return gl_helper_.RunInGlContext([this]() -> absl::Status {
    if (program_) glDeleteProgram(program_);
    program_ = 0;
    if (mask_texture_) glDeleteTextures(1, &mask_texture_);
    mask_texture_ = 0;
    return absl::OkStatus();
  });
}

}  // namespace guideline
