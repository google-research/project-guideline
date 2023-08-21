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

// Renders a camera image to the screen using OpenGL textures. This is capable
// of using an external texture when running on an Android device (more
// efficient), or converting an ARCore image to a texture when running on a
// workstation.

#ifndef PROJECT_GUIDELINE_VISUALIZATION__CAMERA_FEED_RENDERER_H_
#define PROJECT_GUIDELINE_VISUALIZATION__CAMERA_FEED_RENDERER_H_

#include <cstdint>
#include <memory>

// clang-format off
// gl3 must be before gl2ext
#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>  // keep include
// clang-format on
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "Eigen/Core"
#include "project_guideline/util/image.h"

namespace guideline::visualization {

class CameraFeedRenderer {
 public:
  CameraFeedRenderer(bool use_gl_texture, int texture_rotation_degrees = 0,
                     float texture_aspect_ratio = 4.0 / 3.0);

  void SetViewport(int width, int height);
  void Render();

  absl::Status OnGlInit();
  void OnGlTextureUpdated();
  void OnGlTextureReset();
  uint32_t gl_texture_id();

  void OnImage(const std::shared_ptr<const util::Image>& image);

 private:
  const bool use_gl_texture_;
  const int texture_rotation_degrees_;
  float texture_aspect_ratio_;

  absl::Mutex mutex_;
  std::shared_ptr<const util::Image> texture_update_image_
      ABSL_GUARDED_BY(mutex_) = nullptr;
  bool has_valid_texture_ ABSL_GUARDED_BY(mutex_) = false;
  Eigen::Matrix4f transform_matrix_ ABSL_GUARDED_BY(mutex_);

  GLuint texture_;
  GLuint program_;
  GLuint vertex_buffer_;
  GLuint texture_coords_buffer_;
  GLuint vertex_array_;
  GLint uniform_matrix_;
};

}  // namespace guideline::visualization

#endif  // PROJECT_GUIDELINE_VISUALIZATION__CAMERA_FEED_RENDERER_H_
