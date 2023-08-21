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

#ifndef PROJECT_GUIDELINE_VISUALIZATION_ENVIRONMENT_MAP_RENDERER_H_
#define PROJECT_GUIDELINE_VISUALIZATION_ENVIRONMENT_MAP_RENDERER_H_

#include <GLES3/gl3.h>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "Eigen/Core"
#include "project_guideline/environment/control_signal.h"
#include "project_guideline/util/transformation.h"

namespace guideline::visualization {

// Renders a map of the environment (guideline, obstacles, runner trajectory,
// etc) using OpenGL.
class EnvironmentMapRenderer {
 public:
  EnvironmentMapRenderer();
  ~EnvironmentMapRenderer() = default;

  void InitGL();
  void SetViewport(int width, int height);
  void Render();
  void Reset();

  void OnPose(const util::Transformation& position_and_direction);
  void OnControlSignal(const environment::ControlSignal& signal);
  void OnGuideline(const std::vector<Eigen::Vector3d>& guideline);
  void OnObstacles(const std::vector<Eigen::Vector2d>& obstacles);

 private:
  absl::Mutex mutex_;
  environment::ControlSignal control_signal_ ABSL_GUARDED_BY(mutex_);

  std::optional<util::Transformation> position_and_direction_
      ABSL_GUARDED_BY(mutex_);
  std::vector<Eigen::Vector3d> guideline_ ABSL_GUARDED_BY(mutex_);
  std::vector<Eigen::Vector2d> obstacles_ ABSL_GUARDED_BY(mutex_);

  Eigen::Vector2d viewport_;
  Eigen::Vector4d map_viewport_;

  GLuint program_;
  GLuint vertex_buffer_;
  GLuint texture_coords_buffer_;
  GLuint vertex_array_;

  GLint uniform_matrix_;
  GLint uniform_viewport_;
  GLint uniform_runner_position_;
  GLint uniform_runner_vector_;
  GLint uniform_line_points_;
  GLint uniform_line_point_count_;
  GLint uniform_obstacle_points_;
  GLint uniform_obstacle_point_count_;
  GLint uniform_rotation_movement_radians_;
  GLint uniform_lateral_movement_meters_;
  GLint uniform_turn_point_;
  GLint uniform_turn_angle_radians_;
  GLint uniform_stop_;
  GLint uniform_target_point_;
  GLint uniform_target_rotation_movement_radians_;
  GLint uniform_use_target_point_;
};

}  // namespace guideline::visualization

#endif  // PROJECT_GUIDELINE_VISUALIZATION_ENVIRONMENT_MAP_RENDERER_H_
