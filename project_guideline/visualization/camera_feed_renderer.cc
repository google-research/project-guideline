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

#include "project_guideline/visualization/camera_feed_renderer.h"

#include <memory>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "project_guideline/visualization/gl_util.h"

namespace guideline::visualization {

namespace {
using ::Eigen::Matrix4f;

#ifdef __ANDROID__
#define REQUIRE_IMAGE_EXT \
  "#extension GL_OES_EGL_image_external_essl3 : require\n"
#define TEXTURE_TARGET GL_TEXTURE_EXTERNAL_OES
#else
#define REQUIRE_IMAGE_EXT ""
#define TEXTURE_TARGET GL_TEXTURE_2D
#endif

// clang-format off
const GLchar* const kFragmentShader = SHADER_WITH_EXT(REQUIRE_IMAGE_EXT,
    precision mediump float;
#ifdef __ANDROID__
    uniform samplerExternalOES cameraFeedTexture;
#else
    uniform sampler2D cameraFeedTexture;
#endif
    in vec2 textureCoords;
    out vec4 diffuseColor;
    void main() {
      diffuseColor = texture(cameraFeedTexture, textureCoords);
    });

const GLchar* const kVertexShader = SHADER(
    precision mediump float;
    layout(location = 0) in vec4 aPositionCoords;
    layout(location = 1) in vec2 aTextureCoords;
    uniform mat4 uMatrix;
    out vec2 textureCoords;
    void main() {
      gl_Position = uMatrix * aPositionCoords;
      textureCoords = aTextureCoords;
    });
// clang-format on

static const GLfloat kFullVertexCoords[] = {
    -1.0f, -1.0f,  // bottom left
    1.0f,  -1.0f,  // bottom right
    -1.0f, 1.0f,   // top left
    1.0f,  1.0f,   // top right
};

static const GLfloat kFullTextureCoords[] = {
    0.0f, 1.0f,  // bottom left
    1.0f, 1.0f,  // bottom right
    0.0f, 0.0f,  // top left
    1.0f, 0.0f,  // top right
};

}  // namespace

CameraFeedRenderer::CameraFeedRenderer(const bool use_gl_texture,
                                       const int texture_rotation_degrees,
                                       const float texture_aspect_ratio)
    : use_gl_texture_(use_gl_texture),
      texture_rotation_degrees_(texture_rotation_degrees),
      texture_aspect_ratio_(texture_aspect_ratio),
      transform_matrix_(Eigen::Matrix4f::Identity()) {}

absl::Status CameraFeedRenderer::OnGlInit() {
  glGenTextures(1, &texture_);

  glBindTexture(TEXTURE_TARGET, texture_);
  glTexParameteri(TEXTURE_TARGET, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(TEXTURE_TARGET, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(TEXTURE_TARGET, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(TEXTURE_TARGET, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  if (TEXTURE_TARGET == GL_TEXTURE_2D) {
    glTexImage2D(TEXTURE_TARGET, 0, GL_RGB, 512, 512, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, nullptr);
  }
  glBindTexture(TEXTURE_TARGET, 0);
  CheckGlError("set texture params");

  program_ = glCreateProgram();
  CheckGlError("Create camera feed program");

  GLuint vertex_shader =
      LoadShader(GL_VERTEX_SHADER, "cameraFeedVertexShader", kVertexShader);
  glAttachShader(program_, vertex_shader);
  CheckGlError("attach vertex shader");

  GLuint fragment_shader = LoadShader(
      GL_FRAGMENT_SHADER, "cameraFeedFragmentShader", kFragmentShader);
  glAttachShader(program_, fragment_shader);
  CheckGlError("attach fragment shader");

  LinkProgram(program_);

  glDeleteShader(vertex_shader);
  glDeleteShader(fragment_shader);

  glGenVertexArrays(1, &vertex_array_);
  glGenBuffers(1, &vertex_buffer_);
  glGenBuffers(1, &texture_coords_buffer_);

  glBindVertexArray(vertex_array_);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
  glBufferData(GL_ARRAY_BUFFER, 4 * 2 * sizeof(GLfloat), kFullVertexCoords,
               GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glBindBuffer(GL_ARRAY_BUFFER, texture_coords_buffer_);
  glBufferData(GL_ARRAY_BUFFER, 4 * 2 * sizeof(GLfloat), kFullTextureCoords,
               GL_STATIC_DRAW);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  uniform_matrix_ = glGetUniformLocation(program_, "uMatrix");

  CheckGlError("init shaders");

  return absl::OkStatus();
}

void CameraFeedRenderer::OnGlTextureUpdated() {
  absl::MutexLock lock(&mutex_);
  has_valid_texture_ = true;
}

void CameraFeedRenderer::OnGlTextureReset() {
  absl::MutexLock lock(&mutex_);
  has_valid_texture_ = false;
}

uint32_t CameraFeedRenderer::gl_texture_id() {
  CHECK(texture_) << "Must call OnGlInit first";
  return texture_;
}

void CameraFeedRenderer::SetViewport(int width, int height) {
  float aspect_ratio = 1.0 * width / height;

  float x_scale = 1.0;
  float y_scale = 1.0;

  if (aspect_ratio > texture_aspect_ratio_) {
    float desired_width = height * texture_aspect_ratio_;
    x_scale = desired_width / width;
  } else {
    float desired_height = width / texture_aspect_ratio_;
    y_scale = desired_height / height;
  }

  static const float kDeg2Rad = M_PI / 180.f;
  Eigen::Affine3f t = Eigen::Affine3f::Identity();
  t.prerotate(Eigen::AngleAxisf(texture_rotation_degrees_ * kDeg2Rad,
                                Eigen::Vector3f::UnitZ()))
      .prescale(Eigen::Vector3f{x_scale, y_scale, 1.0f});

  {
    absl::MutexLock lock(&mutex_);
    transform_matrix_ = t.matrix();
  }
}

void CameraFeedRenderer::OnImage(
    const std::shared_ptr<const util::Image>& image) {
  absl::MutexLock lock(&mutex_);

  CHECK(image->format() == util::ImageFormat::kRGB)
      << "Image must be RGB format";

  texture_update_image_ = image;
}

void CameraFeedRenderer::Render() {
  Eigen::Matrix4f transform_matrix;
  {
    absl::MutexLock lock(&mutex_);

    if (texture_update_image_) {
      glBindTexture(TEXTURE_TARGET, texture_);
      glTexImage2D(TEXTURE_TARGET, 0, GL_RGB, texture_update_image_->width(),
                   texture_update_image_->height(), 0, GL_RGB, GL_UNSIGNED_BYTE,
                   texture_update_image_->data().ptr<uint8_t>());
      glBindTexture(TEXTURE_TARGET, 0);
      CheckGlError("update camera texture");
      has_valid_texture_ = true;
      texture_update_image_ = nullptr;
    }

    if (!has_valid_texture_) {
      return;
    }

    transform_matrix = transform_matrix_;
  }

  glUseProgram(program_);
  CheckGlError("use program");

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(TEXTURE_TARGET, texture_);
  CheckGlError("bind texture");

  glUniformMatrix4fv(uniform_matrix_, 1, false, transform_matrix.data());

  glBindVertexArray(vertex_array_);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  CheckGlError("bind vertex");

  glBindBuffer(GL_ARRAY_BUFFER, texture_coords_buffer_);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  CheckGlError("bind texture buffer");

  glDisable(GL_DEPTH_TEST);
  glDepthMask(false);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  CheckGlError("draw arrays");

  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glDepthMask(true);
  glEnable(GL_DEPTH_TEST);
  glBindVertexArray(0);
  glBindTexture(TEXTURE_TARGET, 0);
  glUseProgram(0);

  CheckGlError("render camera feed");
}

}  // namespace guideline::visualization
