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

#ifndef PROJECT_GUIDELINE_VISUALIZATION_ML_OUTPUT_RENDERER_H_
#define PROJECT_GUIDELINE_VISUALIZATION_ML_OUTPUT_RENDERER_H_

#include <cstdint>
#include <memory>
#include <vector>

// clang-format off
// gl3 must be before gl2ext
#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>  // keep include
// clang-format on
#include <opencv2/core/mat.hpp>
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "Eigen/Core"
#include "project_guideline/util/image.h"

namespace guideline::visualization {

// Renders an overlay of ML output (segmentation mask + depth map).
class MlOutputRenderer {
 public:
  MlOutputRenderer(int mask_rotation_degrees);
  absl::Status OnGlInit();

  void OnSegmentationMask(std::shared_ptr<const util::ConfidenceMask> mask,
                          const std::vector<Eigen::Vector3f>& keypoints);
  void OnDepthMap(std::shared_ptr<const util::DepthImage> depth);

  void SetViewport(int width, int height);
  void Render();

 private:
  const int mask_rotation_degrees_;

  absl::Mutex mutex_;
  Eigen::Matrix4f transform_matrix_ ABSL_GUARDED_BY(mutex_);
  std::shared_ptr<const util::ConfidenceMask> segmentation_mask_
      ABSL_GUARDED_BY(mutex_) = nullptr;
  std::vector<Eigen::Vector3f> keypoints_ ABSL_GUARDED_BY(mutex_);

  std::shared_ptr<const util::DepthImage> depth_map_ ABSL_GUARDED_BY(mutex_) =
      nullptr;

  int viewport_width_ ABSL_GUARDED_BY(mutex_) = 0;
  int viewport_height_ ABSL_GUARDED_BY(mutex_) = 0;

  GLuint mask_texture_;
  GLuint depth_texture_;
  GLuint depth_colormap_texture_;
  GLuint program_;
  GLuint vertex_buffer_;
  GLuint texture_coords_buffer_;
  GLuint vertex_array_;
  GLint uniform_matrix_;
  GLint uniform_mask_texture_;
  GLint uniform_depth_texture_;
  GLint uniform_depth_colormap_;

  GLuint uniform_keypoint_count_;
  GLuint uniform_keypoints_;
};

}  // namespace guideline::visualization

#endif  // PROJECT_GUIDELINE_VISUALIZATION_ML_OUTPUT_RENDERER_H_
