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

#include "project_guideline/visualization/ml_output_renderer.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <GLES3/gl3.h>
#include <opencv2/core/mat.hpp>
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "project_guideline/util/image.h"
#include "project_guideline/visualization/gl_util.h"

namespace guideline::visualization {

namespace {
using ::Eigen::Matrix4f;

constexpr int kMaskTextureUnit = 0;
constexpr int kDepthTextureUnit = 1;
constexpr int kDepthColormapTextureUnit = 2;

// clang-format off
const GLchar* const kFragmentShader = SHADER(
    precision mediump float;
    uniform sampler2D maskTexture;
    uniform sampler2D depthTexture;
    uniform sampler2D depthColormap;
    uniform vec2 uKeypoints[100];
    uniform int uKeypointCount;
    in vec2 textureCoords;
    out vec4 diffuseColor;

    const float KEYPOINT_RADIUS = 0.005;
    const vec4 KEYPOINT_PRIMARY_COLOR = vec4(1, 0, 0, 1.0);
    const vec4 KEYPOINT_SECONDARY_COLOR = vec4(0, 0, 1, 1.0);

    const float MIN_DEPTH = 0.01;
    const float MAX_DEPTH = 5.0;

    vec4 blend(vec4 under, vec4 over) {
      vec4 result = mix(under, over, over.a);
      result.a = over.a + under.a * (1.0 - over.a);
      return result;
    }

    vec4 keypoints(vec2 position) {
      for (int i = 0; i < uKeypointCount; i++) {
        if (distance(position, uKeypoints[i]) < KEYPOINT_RADIUS) {
          return mix(KEYPOINT_PRIMARY_COLOR, KEYPOINT_SECONDARY_COLOR, float(i) / float(uKeypointCount));
        }
      }
      return vec4(0, 0, 0, 0);
    }

    void main() {
      // Render the line segmentation mask as green, with alpha based on
      // the confidence value.
      float maskTexel = float(texture(maskTexture, textureCoords));
      vec4 maskColor = vec4(0., 1., 0., maskTexel * 0.6);

      // Get the depth value (meters) and scale it to range [0, 1], then
      // use that to get a color from the colormap texture.
      float depthTexel = float(texture(depthTexture, textureCoords));
      float depthLevel = (depthTexel - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH);
      depthLevel = clamp(depthLevel, 0., 1.);
      vec4 depthColor = texture(depthColormap, vec2(1.0 - depthLevel, 0.));
      depthColor.w = depthTexel > MIN_DEPTH ? 0.6 : 0.0;

      diffuseColor = blend(depthColor, maskColor);
      diffuseColor = blend(diffuseColor, keypoints(textureCoords));
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

MlOutputRenderer::MlOutputRenderer(int mask_rotation_degrees)
    : mask_rotation_degrees_(mask_rotation_degrees),
      transform_matrix_(Eigen::Matrix4f::Identity()) {}

absl::Status MlOutputRenderer::OnGlInit() {
  glGenTextures(1, &mask_texture_);
  glBindTexture(GL_TEXTURE_2D, mask_texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_2D, 0);

  glGenTextures(1, &depth_texture_);
  glBindTexture(GL_TEXTURE_2D, depth_texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_2D, 0);
  CheckGlError("set texture params");

  glGenTextures(1, &depth_colormap_texture_);
  glBindTexture(GL_TEXTURE_2D, depth_colormap_texture_);

  // An array of R,G,B values used for the depth color map. The fragment shader
  // will map values between [0,1] to a color value in this list.
  static const uint8_t kDepthColorMap[] = {
      0x44, 0x01, 0x54, 0x44, 0x02, 0x55, 0x45, 0x03, 0x57, 0x45, 0x05, 0x58,
      0x45, 0x06, 0x5A, 0x46, 0x08, 0x5B, 0x46, 0x09, 0x5D, 0x46, 0x0B, 0x5E,
      0x46, 0x0C, 0x60, 0x47, 0x0E, 0x61, 0x47, 0x0F, 0x62, 0x47, 0x11, 0x64,
      0x47, 0x12, 0x65, 0x47, 0x14, 0x66, 0x48, 0x15, 0x68, 0x48, 0x16, 0x69,
      0x48, 0x18, 0x6A, 0x48, 0x19, 0x6C, 0x48, 0x1A, 0x6D, 0x48, 0x1C, 0x6E,
      0x48, 0x1D, 0x6F, 0x48, 0x1E, 0x70, 0x48, 0x20, 0x71, 0x48, 0x21, 0x73,
      0x48, 0x22, 0x74, 0x48, 0x24, 0x75, 0x48, 0x25, 0x76, 0x48, 0x26, 0x77,
      0x48, 0x27, 0x78, 0x47, 0x29, 0x79, 0x47, 0x2A, 0x79, 0x47, 0x2B, 0x7A,
      0x47, 0x2C, 0x7B, 0x47, 0x2E, 0x7C, 0x46, 0x2F, 0x7D, 0x46, 0x30, 0x7E,
      0x46, 0x31, 0x7E, 0x46, 0x33, 0x7F, 0x45, 0x34, 0x80, 0x45, 0x35, 0x81,
      0x45, 0x36, 0x81, 0x44, 0x38, 0x82, 0x44, 0x39, 0x83, 0x44, 0x3A, 0x83,
      0x43, 0x3B, 0x84, 0x43, 0x3C, 0x84, 0x43, 0x3E, 0x85, 0x42, 0x3F, 0x85,
      0x42, 0x40, 0x86, 0x41, 0x41, 0x86, 0x41, 0x42, 0x87, 0x41, 0x43, 0x87,
      0x40, 0x45, 0x88, 0x40, 0x46, 0x88, 0x3F, 0x47, 0x88, 0x3F, 0x48, 0x89,
      0x3E, 0x49, 0x89, 0x3E, 0x4A, 0x89, 0x3D, 0x4B, 0x8A, 0x3D, 0x4D, 0x8A,
      0x3C, 0x4E, 0x8A, 0x3C, 0x4F, 0x8A, 0x3B, 0x50, 0x8B, 0x3B, 0x51, 0x8B,
      0x3A, 0x52, 0x8B, 0x3A, 0x53, 0x8B, 0x39, 0x54, 0x8C, 0x39, 0x55, 0x8C,
      0x38, 0x56, 0x8C, 0x38, 0x57, 0x8C, 0x37, 0x58, 0x8C, 0x37, 0x59, 0x8C,
      0x36, 0x5B, 0x8D, 0x36, 0x5C, 0x8D, 0x35, 0x5D, 0x8D, 0x35, 0x5E, 0x8D,
      0x34, 0x5F, 0x8D, 0x34, 0x60, 0x8D, 0x33, 0x61, 0x8D, 0x33, 0x62, 0x8D,
      0x33, 0x63, 0x8D, 0x32, 0x64, 0x8E, 0x32, 0x65, 0x8E, 0x31, 0x66, 0x8E,
      0x31, 0x67, 0x8E, 0x30, 0x68, 0x8E, 0x30, 0x69, 0x8E, 0x2F, 0x6A, 0x8E,
      0x2F, 0x6B, 0x8E, 0x2F, 0x6C, 0x8E, 0x2E, 0x6D, 0x8E, 0x2E, 0x6E, 0x8E,
      0x2D, 0x6F, 0x8E, 0x2D, 0x70, 0x8E, 0x2D, 0x70, 0x8E, 0x2C, 0x71, 0x8E,
      0x2C, 0x72, 0x8E, 0x2B, 0x73, 0x8E, 0x2B, 0x74, 0x8E, 0x2B, 0x75, 0x8E,
      0x2A, 0x76, 0x8E, 0x2A, 0x77, 0x8E, 0x29, 0x78, 0x8E, 0x29, 0x79, 0x8E,
      0x29, 0x7A, 0x8E, 0x28, 0x7B, 0x8E, 0x28, 0x7C, 0x8E, 0x28, 0x7D, 0x8E,
      0x27, 0x7E, 0x8E, 0x27, 0x7F, 0x8E, 0x26, 0x80, 0x8E, 0x26, 0x81, 0x8E,
      0x26, 0x82, 0x8E, 0x25, 0x83, 0x8E, 0x25, 0x83, 0x8E, 0x25, 0x84, 0x8E,
      0x24, 0x85, 0x8E, 0x24, 0x86, 0x8E, 0x23, 0x87, 0x8E, 0x23, 0x88, 0x8E,
      0x23, 0x89, 0x8E, 0x22, 0x8A, 0x8D, 0x22, 0x8B, 0x8D, 0x22, 0x8C, 0x8D,
      0x21, 0x8D, 0x8D, 0x21, 0x8E, 0x8D, 0x21, 0x8F, 0x8D, 0x20, 0x90, 0x8D,
      0x20, 0x91, 0x8C, 0x20, 0x92, 0x8C, 0x20, 0x93, 0x8C, 0x1F, 0x93, 0x8C,
      0x1F, 0x94, 0x8C, 0x1F, 0x95, 0x8B, 0x1F, 0x96, 0x8B, 0x1F, 0x97, 0x8B,
      0x1E, 0x98, 0x8B, 0x1E, 0x99, 0x8A, 0x1E, 0x9A, 0x8A, 0x1E, 0x9B, 0x8A,
      0x1E, 0x9C, 0x89, 0x1E, 0x9D, 0x89, 0x1E, 0x9E, 0x89, 0x1E, 0x9F, 0x88,
      0x1E, 0xA0, 0x88, 0x1F, 0xA1, 0x88, 0x1F, 0xA2, 0x87, 0x1F, 0xA3, 0x87,
      0x1F, 0xA3, 0x86, 0x20, 0xA4, 0x86, 0x20, 0xA5, 0x86, 0x21, 0xA6, 0x85,
      0x21, 0xA7, 0x85, 0x22, 0xA8, 0x84, 0x23, 0xA9, 0x83, 0x23, 0xAA, 0x83,
      0x24, 0xAB, 0x82, 0x25, 0xAC, 0x82, 0x26, 0xAD, 0x81, 0x27, 0xAE, 0x81,
      0x28, 0xAF, 0x80, 0x29, 0xAF, 0x7F, 0x2A, 0xB0, 0x7F, 0x2B, 0xB1, 0x7E,
      0x2C, 0xB2, 0x7D, 0x2E, 0xB3, 0x7C, 0x2F, 0xB4, 0x7C, 0x30, 0xB5, 0x7B,
      0x32, 0xB6, 0x7A, 0x33, 0xB7, 0x79, 0x35, 0xB7, 0x79, 0x36, 0xB8, 0x78,
      0x38, 0xB9, 0x77, 0x39, 0xBA, 0x76, 0x3B, 0xBB, 0x75, 0x3D, 0xBC, 0x74,
      0x3E, 0xBD, 0x73, 0x40, 0xBE, 0x72, 0x42, 0xBE, 0x71, 0x44, 0xBF, 0x70,
      0x46, 0xC0, 0x6F, 0x48, 0xC1, 0x6E, 0x49, 0xC2, 0x6D, 0x4B, 0xC2, 0x6C,
      0x4D, 0xC3, 0x6B, 0x4F, 0xC4, 0x6A, 0x51, 0xC5, 0x69, 0x53, 0xC6, 0x68,
      0x55, 0xC6, 0x66, 0x58, 0xC7, 0x65, 0x5A, 0xC8, 0x64, 0x5C, 0xC9, 0x63,
      0x5E, 0xC9, 0x62, 0x60, 0xCA, 0x60, 0x62, 0xCB, 0x5F, 0x65, 0xCC, 0x5E,
      0x67, 0xCC, 0x5C, 0x69, 0xCD, 0x5B, 0x6C, 0xCE, 0x5A, 0x6E, 0xCE, 0x58,
      0x70, 0xCF, 0x57, 0x73, 0xD0, 0x55, 0x75, 0xD0, 0x54, 0x77, 0xD1, 0x52,
      0x7A, 0xD2, 0x51, 0x7C, 0xD2, 0x4F, 0x7F, 0xD3, 0x4E, 0x81, 0xD4, 0x4C,
      0x84, 0xD4, 0x4B, 0x86, 0xD5, 0x49, 0x89, 0xD5, 0x48, 0x8B, 0xD6, 0x46,
      0x8E, 0xD7, 0x44, 0x90, 0xD7, 0x43, 0x93, 0xD8, 0x41, 0x95, 0xD8, 0x3F,
      0x98, 0xD9, 0x3E, 0x9B, 0xD9, 0x3C, 0x9D, 0xDA, 0x3A, 0xA0, 0xDA, 0x39,
      0xA3, 0xDB, 0x37, 0xA5, 0xDB, 0x35, 0xA8, 0xDC, 0x33, 0xAB, 0xDC, 0x32,
      0xAD, 0xDD, 0x30, 0xB0, 0xDD, 0x2E, 0xB3, 0xDD, 0x2D, 0xB5, 0xDE, 0x2B,
      0xB8, 0xDE, 0x29, 0xBB, 0xDF, 0x27, 0xBD, 0xDF, 0x26, 0xC0, 0xDF, 0x24,
      0xC3, 0xE0, 0x23, 0xC5, 0xE0, 0x21, 0xC8, 0xE1, 0x20, 0xCB, 0xE1, 0x1E,
      0xCD, 0xE1, 0x1D, 0xD0, 0xE2, 0x1C, 0xD3, 0xE2, 0x1B, 0xD5, 0xE2, 0x1A,
      0xD8, 0xE3, 0x19, 0xDB, 0xE3, 0x18, 0xDD, 0xE3, 0x18, 0xE0, 0xE4, 0x18,
      0xE2, 0xE4, 0x18, 0xE5, 0xE4, 0x18, 0xE8, 0xE5, 0x19, 0xEA, 0xE5, 0x19,
      0xED, 0xE5, 0x1A, 0xEF, 0xE6, 0x1B, 0xF2, 0xE6, 0x1C, 0xF4, 0xE6, 0x1E,
      0xF7, 0xE6, 0x1F, 0xF9, 0xE7, 0x21, 0xFB, 0xE7, 0x23, 0xFE, 0xE7, 0x24};

  // Bind the colormap as a 2D texture with height=1 (essentially a 1d texture).
  // A value is selected by sampling the texture at x=[0,1] and y=0.
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 256, 1, 0, GL_RGB, GL_UNSIGNED_BYTE,
               kDepthColorMap);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  program_ = glCreateProgram();
  CheckGlError("Create ML output program");

  GLuint vertex_shader =
      LoadShader(GL_VERTEX_SHADER, "mlOutputVertexShader", kVertexShader);
  glAttachShader(program_, vertex_shader);
  CheckGlError("attach vertex shader");

  GLuint fragment_shader =
      LoadShader(GL_FRAGMENT_SHADER, "mlOutputFragmentShader", kFragmentShader);
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
  uniform_mask_texture_ = glGetUniformLocation(program_, "maskTexture");
  uniform_depth_texture_ = glGetUniformLocation(program_, "depthTexture");
  uniform_depth_colormap_ = glGetUniformLocation(program_, "depthColormap");

  uniform_keypoint_count_ = glGetUniformLocation(program_, "uKeypointCount");
  uniform_keypoints_ = glGetUniformLocation(program_, "uKeypoints");

  CheckGlError("init shaders");

  return absl::OkStatus();
}

void MlOutputRenderer::SetViewport(int width, int height) {
  // The camera preview uses 16:9 aspect ratio, while the CPU image used for
  // processing in the pipeline is the full 4:3 sensor image.
  float kPreviewAspectRatio;
  float kCpuImageAspectRatio;
  if (width < height) {
    kPreviewAspectRatio = 9.0f / 16.0f;
    kCpuImageAspectRatio = 3.0f / 4.0f;
  } else {
    kPreviewAspectRatio = 16.0f / 9.0f;
    kCpuImageAspectRatio = 4.0f / 3.0f;
  }

  // The actual width of the preview image.
  float preview_width = height * kPreviewAspectRatio;

  // By default the mask texture is rendered to fit across the full viewport
  // (aspect ratio not preserved).
  // Scale the width matches the preview image, and scale the height so it
  // maintains the aspect ratio of the 4:3 input image (some of the top/bottom
  // will be off-screen). The mask will then be properly aligned with the
  // preview image.
  float x_scale = preview_width / width;
  float desired_mask_height = preview_width / kCpuImageAspectRatio;
  float y_scale = desired_mask_height / height;

  static const float kDeg2Rad = M_PI / 180.f;
  Eigen::Affine3f t = Eigen::Affine3f::Identity();
  t.prerotate(Eigen::AngleAxisf(mask_rotation_degrees_ * kDeg2Rad,
                                Eigen::Vector3f::UnitZ()))
      .prescale(Eigen::Vector3f{x_scale, y_scale, 1.0f});

  {
    absl::MutexLock lock(&mutex_);
    transform_matrix_ = t.matrix();
  }
}

void MlOutputRenderer::OnSegmentationMask(
    std::shared_ptr<const util::ConfidenceMask> segmentation_mask,
    const std::vector<Eigen::Vector3f>& keypoints) {
  absl::MutexLock lock(&mutex_);
  segmentation_mask_ = segmentation_mask;
  keypoints_ = keypoints;
}

void MlOutputRenderer::OnDepthMap(
    std::shared_ptr<const util::DepthImage> depth_map) {
  absl::MutexLock lock(&mutex_);
  depth_map_ = depth_map;
}

void MlOutputRenderer::Render() {
  // This directly binds the float data from the segmentation mask and depth
  // map buffers to a gl textures. The fragment shader then uses the texture
  // data to render these to the screen.

  Eigen::Matrix4f transform_matrix;
  {
    absl::MutexLock lock(&mutex_);
    if (segmentation_mask_) {
      glBindTexture(GL_TEXTURE_2D, mask_texture_);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, segmentation_mask_->width(),
                   segmentation_mask_->height(), 0, GL_RED, GL_FLOAT,
                   segmentation_mask_->data().ptr<float>());
      glBindTexture(GL_TEXTURE_2D, 0);
      CheckGlError("update mask texture");
    }
    if (depth_map_) {
      glBindTexture(GL_TEXTURE_2D, depth_texture_);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, depth_map_->width(),
                   depth_map_->height(), 0, GL_RED, GL_FLOAT,
                   depth_map_->data().ptr<float>());
      glBindTexture(GL_TEXTURE_2D, 0);
      CheckGlError("update depth texture");
    }
    transform_matrix = transform_matrix_;
  }

  glUseProgram(program_);

  {
    absl::MutexLock lock(&mutex_);
    if (!keypoints_.empty()) {
      const size_t kMaxKeypointCount = 100;
      if (keypoints_.size() > kMaxKeypointCount) {
        keypoints_.resize(kMaxKeypointCount);
      }
      glUniform1i(uniform_keypoint_count_, keypoints_.size());
      CheckGlError("update keypoint count");
      std::vector<GLfloat> gl_points;
      for (const auto& point : keypoints_) {
        gl_points.push_back(point.x());
        gl_points.push_back(point.y());
      }
      glUniform2fv(uniform_keypoints_, gl_points.size(), gl_points.data());
      CheckGlError("update keypoints data");
    } else {
      glUniform1i(uniform_keypoint_count_, 0);
      CheckGlError("no keypoints");
    }
    CheckGlError("update keypoints");
  }
  CheckGlError("use program");

  glUniform1i(uniform_mask_texture_, kMaskTextureUnit);
  glUniform1i(uniform_depth_texture_, kDepthTextureUnit);
  glUniform1i(uniform_depth_colormap_, kDepthColormapTextureUnit);

  glActiveTexture(GL_TEXTURE0 + kMaskTextureUnit);
  glBindTexture(GL_TEXTURE_2D, mask_texture_);
  CheckGlError("bind mask texture");

  glActiveTexture(GL_TEXTURE0 + kDepthTextureUnit);
  glBindTexture(GL_TEXTURE_2D, depth_texture_);
  CheckGlError("bind depth texture");

  glActiveTexture(GL_TEXTURE0 + kDepthColormapTextureUnit);
  glBindTexture(GL_TEXTURE_2D, depth_colormap_texture_);
  CheckGlError("bind depth colormap texture");

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
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  CheckGlError("draw arrays");

  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glDepthMask(true);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glBindVertexArray(0);
  glBindTexture(GL_TEXTURE_2D, 0);
  glUseProgram(0);

  CheckGlError("render segmentation mask");
}

}  // namespace guideline::visualization
