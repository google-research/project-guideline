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

#include "project_guideline/visualization/environment_map_renderer.h"

#include <optional>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/transformation.h"
#include "project_guideline/visualization/gl_util.h"

namespace guideline::visualization {

namespace {

// clang-format off
const GLchar* const kVertexShader = SHADER(
    precision mediump float;
    layout(location = 0) in vec4 aPositionCoords;
    layout(location = 1) in vec2 aTextureCoords;
    out vec2 textureCoords;
    void main() {
      gl_Position = aPositionCoords;
      textureCoords = aTextureCoords;
    });

// A fragment shader is used to render the environment map. Note that this is
// pushing the limits of what a fragment shader can do.
// TODO: Re-implement using basic OpenGL geometric elements and shaders.
const GLchar* const kFragmentShader = SHADER(
    precision mediump float;

    layout(location = 0) out vec4 diffuseColor;
    uniform vec4 uViewport;
    uniform vec2 uRunnerPosition;
    uniform vec2 uRunnerVector;
    uniform float uRotationMovementRadians;
    uniform float uLateralMovementMeters;
    uniform vec2 uTurnPoint;
    uniform float uTurnAngleRadians;
    uniform bool uStop;
    uniform vec2 uLinePoints[100];
    uniform int uLinePointCount;
    uniform vec2 uTargetPoint;
    uniform float uTargetRotationMovementRadians;
    uniform bool uUseTargetPoint;
    uniform mat4 uMatrix;
    uniform vec2 uObstaclePoints[100];
    uniform int uObstaclePointCount;

    const float M_SQRT_2 = 1.41421356237;
    const float MAP_RADIUS_METERS = 15.0;
    const float MAP_SIZE_METERS = MAP_RADIUS_METERS * 2.;
    const vec2 CENTER = vec2(0.5, 0.5);

    const vec4 ORIGIN_POINT_COLOR = vec4(0., 0., 1., .7);
    const vec4 MAP_BACKGROUND_COLOR = vec4(0.5, 0.5, 0.5, 0.1);
    const vec4 TRANSPARENT = vec4(0., 0., 0., 0.);

    const float GRID_SIZE_METERS = 1.0;
    const float GRID_LINE_PITCH  = GRID_SIZE_METERS / MAP_SIZE_METERS;
    const float GRID_LINE_WIDTH = 0.007;
    const float GRID_LINE_ANTIALIAS = 1.2;
    const vec4 GRID_LINE_COLOR = vec4(0.2, 0.3, 0.6, 0.7);

    const float GUIDELINE_POINT_RADIUS = 0.02;
    const vec4 GUIDELINE_POINT_COLOR = vec4(1., 0.3, 0.3, 0.7);
    const vec4 GUIDELINE_ARROW_COLOR = vec4(0.4, 0.1, 0.1, 0.7);
    const vec4 TARGET_POINT_COLOR = vec4(0.8, 0.0, 0.4, 0.9);
    const int GUIDELINE_ARROW_STRIDE = 4;
    const vec4 TURN_POINT_COLOR = vec4(1., 0., 1., 0.7);
    const vec4 TURN_LINE_COLOR = vec4(1., 0.2, 1., 0.7);
    const vec4 STOP_COLOR = vec4(1.0, 0., 0., 0.7);
    const float MOVEMENT_LINE_WIDTH = 0.009;
    const vec4 MOVEMENT_LINE_COLOR = vec4(0.2, 0.8, 0.2, 1.0);
    const vec4 OBSTACLE_POINT_COLOR = vec4(0.99, 0.56, 0.32, 0.7);

    const float VIGNETTE_START = 0.45;
    const float VIGNETTE_END = 0.5;

    vec2 world_point_to_map(vec2 w) {
      w -= uRunnerPosition;
      w /= MAP_SIZE_METERS;
      w += 0.5;
      return w;
    }

    float distance_to_world_point(vec2 p, vec2 w) {
      return distance(p, world_point_to_map(w));
    }

    float angle(vec2 p1, vec2 p2) {
      return atan(p2.x - p1.x, p2.y - p1.y);
    }

    float square(vec2 p, vec2 center, float size) {
      if (max(abs(p.x - center.x), abs(p.y - center.y)) < size) {
        return 1.;
      }
      return 0.;
    }

    mat2 rotation2d(float angle) {
      float s = sin(angle);
      float c = cos(angle);
      return mat2(
        c, -s,
        s, c
      );
    }

    vec2 rotate(vec2 v, float angle) {
      return rotation2d(angle) * v;
    }

    float segment_distance(vec2 p, vec2 p1, vec2 p2) {
      vec2 center = (p1 + p2) * 0.5;
      float len = length(p2 - p1);
      vec2 dir = (p2 - p1) / len;
      vec2 rel_p = p - center;
      float dist1 = abs(dot(rel_p, vec2(dir.y, -dir.x)));
      float dist2 = abs(dot(rel_p, dir)) - 0.5 * len;
      return max(dist1, dist2);
    }

    vec4 obstacle_points(vec2 p) {
      for (int i = 0; i < uObstaclePointCount; i++) {
        vec2 point = uObstaclePoints[i];
        if (distance_to_world_point(p, point) < GUIDELINE_POINT_RADIUS) {
          return OBSTACLE_POINT_COLOR;
        }
      }
      return TRANSPARENT;
    }

    vec4 guideline_points(vec2 p) {
      if (distance(p, CENTER) < GUIDELINE_POINT_RADIUS) {
        return ORIGIN_POINT_COLOR;
      }

      for (int i = 0; i < uLinePointCount; i++) {
        vec2 point = uLinePoints[i];

        if (i % GUIDELINE_ARROW_STRIDE == 0 && i < uLinePointCount - 1) {
          float next_angle = angle(point, uLinePoints[i + 1]);
          vec2 arrow_p1 = vec2(0, 0.01);
          vec2 arrow_p2 = vec2(-0.02, -0.01);
          vec2 arrow_p3 = vec2(0.02, -0.01);
          arrow_p1 = rotate(arrow_p1, next_angle);
          arrow_p2 = rotate(arrow_p2, next_angle);
          arrow_p3 = rotate(arrow_p3, next_angle);
          vec2 world_point = world_point_to_map(point);
          arrow_p1 += world_point;
          arrow_p2 += world_point;
          arrow_p3 += world_point;

          if (segment_distance(p, arrow_p1, arrow_p2) < MOVEMENT_LINE_WIDTH
              || segment_distance(p, arrow_p1, arrow_p3)
                  < MOVEMENT_LINE_WIDTH) {
            return GUIDELINE_ARROW_COLOR;
          }
        }

        if (distance_to_world_point(p, point) < GUIDELINE_POINT_RADIUS) {
          return GUIDELINE_POINT_COLOR;
        }
      }

      return TRANSPARENT;
    }

    vec4 guideline_control(vec2 p, vec2 rp) {
      // Red square for stop signal.
      if (uStop) {
        return square(p, CENTER, 0.1) * STOP_COLOR;
      }

      if (uUseTargetPoint) {
        vec2 movement_p1 = vec2(0., 0.);
        vec2 movement_p2 = vec2(0., 0.2);
        vec2 arrow_p1 = vec2(-0.02, 0.18);
        vec2 arrow_p2 = vec2(0.02, 0.18);
        movement_p1 = rotate(movement_p1, uTargetRotationMovementRadians);
        movement_p2 = rotate(movement_p2, uTargetRotationMovementRadians);
        arrow_p1 = rotate(arrow_p1, uTargetRotationMovementRadians);
        arrow_p2 = rotate(arrow_p2, uTargetRotationMovementRadians);
        movement_p1 += CENTER;
        movement_p2 += CENTER;
        arrow_p1 += CENTER;
        arrow_p2 += CENTER;

        if (segment_distance(p, movement_p1, movement_p2) < MOVEMENT_LINE_WIDTH
           || segment_distance(p, movement_p2, arrow_p1) < MOVEMENT_LINE_WIDTH
           || segment_distance(p, movement_p2, arrow_p2) < MOVEMENT_LINE_WIDTH)
        {
          return MOVEMENT_LINE_COLOR;
        }
      } else {
        // Render line representing lateral and rotational movement.
        float movement_x = (uLateralMovementMeters / MAP_SIZE_METERS);
        vec2 movement_p1 = vec2(movement_x, -0.1);
        vec2 movement_p2 = vec2(movement_x, 0.1);
        vec2 arrow_p1 = vec2(movement_x - 0.02, 0.08);
        vec2 arrow_p2 = vec2(movement_x + 0.02, 0.08);
        movement_p1 = rotate(movement_p1, uRotationMovementRadians);
        movement_p2 = rotate(movement_p2, uRotationMovementRadians);
        arrow_p1 = rotate(arrow_p1, uRotationMovementRadians);
        arrow_p2 = rotate(arrow_p2, uRotationMovementRadians);
        movement_p1 += CENTER;
        movement_p2 += CENTER;
        arrow_p1 += CENTER;
        arrow_p2 += CENTER;

        if (segment_distance(p, movement_p1, movement_p2) < MOVEMENT_LINE_WIDTH
           || segment_distance(p, movement_p2, arrow_p1) < MOVEMENT_LINE_WIDTH
           || segment_distance(p, movement_p2, arrow_p2) < MOVEMENT_LINE_WIDTH)
        {
          return MOVEMENT_LINE_COLOR;
        }

        // Render a circle and line for the turn point and angle.
        if (abs(uTurnAngleRadians) > 0.) {
          if (distance_to_world_point(rp, uTurnPoint) < GUIDELINE_POINT_RADIUS)
          {
            return TURN_POINT_COLOR;
          }

          vec2 turn_p1 = vec2(0, -0.1);
          vec2 turn_p2 = vec2(0, 0.1);
          turn_p1 = rotate(turn_p1, uTurnAngleRadians);
          turn_p2 = rotate(turn_p2, uTurnAngleRadians);
          vec2 turn_point_center = world_point_to_map(uTurnPoint);
          turn_p1 += turn_point_center;
          turn_p2 += turn_point_center;

          if (segment_distance(rp, turn_p1, turn_p2) < MOVEMENT_LINE_WIDTH) {
            return TURN_LINE_COLOR;
          }
        }
      }

      return TRANSPARENT;
    }

    vec4 guideline_target(vec2 p) {
      if (uUseTargetPoint
          && distance_to_world_point(p, uTargetPoint)
              < GUIDELINE_POINT_RADIUS) {
        return TARGET_POINT_COLOR;
      }
      return TRANSPARENT;
    }

    vec4 blend(vec4 under, vec4 over) {
      vec4 result = mix(under, over, over.a);
      result.a = over.a + under.a * (1.0 - over.a);

      return result;
    }

    vec4 draw_grid(vec2 p) {
      vec2 grid_coords = p.xy + (uRunnerPosition / MAP_SIZE_METERS);

      float dist = min(
          mod(grid_coords.x, GRID_LINE_PITCH),
          mod(grid_coords.y, GRID_LINE_PITCH));

      if (dist < GRID_LINE_WIDTH) {
        return GRID_LINE_COLOR * (1.0 -
            smoothstep(GRID_LINE_WIDTH / GRID_LINE_ANTIALIAS,
                       GRID_LINE_WIDTH, dist));
      }

      return TRANSPARENT;
    }

    void main() {
      vec2 screen_coords = gl_FragCoord.xy - uViewport.xy;
      screen_coords.x /= uViewport.z;
      screen_coords.y /= uViewport.w;
      vec2 rotated_screen_coords = (uMatrix * vec4(screen_coords.x,
                                      screen_coords.y, 0., 1.)).xy;

      diffuseColor = MAP_BACKGROUND_COLOR;
      diffuseColor = blend(diffuseColor, draw_grid(rotated_screen_coords));
      diffuseColor = blend(diffuseColor,
                           guideline_points(rotated_screen_coords));
      diffuseColor = blend(diffuseColor,
                           obstacle_points(rotated_screen_coords));
      diffuseColor = blend(diffuseColor,
                           guideline_control(screen_coords,
                                             rotated_screen_coords));
      diffuseColor = blend(diffuseColor,
                           guideline_target(rotated_screen_coords));

      // Apply vignette.
      float dist = distance(screen_coords, CENTER);
      if (dist > VIGNETTE_END) {
        discard;
      } else if (dist > VIGNETTE_START && dist < VIGNETTE_END) {
        diffuseColor.a *=
            (1.0 - smoothstep(VIGNETTE_START, VIGNETTE_END, dist));
      }
    });
// clang-format on

static const GLfloat kFullVertexCoords[] = {
    -1.0f, -1.0f,  // bottom left
    1.0f,  -1.0f,  // bottom right
    -1.0f, 1.0f,   // top left
    1.0f,  1.0f,   // top right
};

static const GLfloat kFullTextureCoords[] = {
    0.0f, 0.0f,  // bottom left
    1.0f, 0.0f,  // bottom right
    0.0f, 1.0f,  // top left
    1.0f, 1.0f,  // top right
};

}  // namespace

EnvironmentMapRenderer::EnvironmentMapRenderer() {
  control_signal_.stop = true;
}

void EnvironmentMapRenderer::InitGL() {
  program_ = glCreateProgram();
  CheckGlError("Create map program");

  GLuint vertex_shader =
      LoadShader(GL_VERTEX_SHADER, "mapVertexShader", kVertexShader);
  glAttachShader(program_, vertex_shader);
  CheckGlError("attach vertex shader");

  GLuint fragment_shader =
      LoadShader(GL_FRAGMENT_SHADER, "mapFragmentShader", kFragmentShader);
  glAttachShader(program_, fragment_shader);
  CheckGlError("attach fragment shader");

  LinkProgram(program_);

  glDeleteShader(vertex_shader);
  glDeleteShader(fragment_shader);

  uniform_viewport_ = glGetUniformLocation(program_, "uViewport");
  uniform_matrix_ = glGetUniformLocation(program_, "uMatrix");
  uniform_runner_position_ = glGetUniformLocation(program_, "uRunnerPosition");
  uniform_runner_vector_ = glGetUniformLocation(program_, "uRunnerVector");
  uniform_line_points_ = glGetUniformLocation(program_, "uLinePoints");
  uniform_line_point_count_ = glGetUniformLocation(program_, "uLinePointCount");
  uniform_obstacle_points_ = glGetUniformLocation(program_, "uObstaclePoints");
  uniform_obstacle_point_count_ =
      glGetUniformLocation(program_, "uObstaclePointCount");
  uniform_rotation_movement_radians_ =
      glGetUniformLocation(program_, "uRotationMovementRadians");
  uniform_lateral_movement_meters_ =
      glGetUniformLocation(program_, "uLateralMovementMeters");
  uniform_turn_point_ = glGetUniformLocation(program_, "uTurnPoint");
  uniform_turn_angle_radians_ =
      glGetUniformLocation(program_, "uTurnAngleRadians");
  uniform_stop_ = glGetUniformLocation(program_, "uStop");
  uniform_target_point_ = glGetUniformLocation(program_, "uTargetPoint");
  uniform_target_rotation_movement_radians_ =
      glGetUniformLocation(program_, "uTargetRotationMovementRadians");
  uniform_use_target_point_ = glGetUniformLocation(program_, "uUseTargetPoint");

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

  CheckGlError("environment map renderer init");
}

void EnvironmentMapRenderer::SetViewport(int width, int height) {
  viewport_ = Eigen::Vector2d(width, height);

  int size = (int)(height * 0.4);
  int margin = (int)(height * 0.05);

  map_viewport_ = Eigen::Vector4d(width - margin - size, margin, size, size);
}

void EnvironmentMapRenderer::Render() {
  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);
  CheckGlError("push viewport");

  glViewport(map_viewport_[0], map_viewport_[1], map_viewport_[2],
             map_viewport_[3]);
  CheckGlError("glviewport");

  glUseProgram(program_);

  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

  glUniform4f(uniform_viewport_, map_viewport_[0], map_viewport_[1],
              map_viewport_[2], map_viewport_[3]);

  absl::MutexLock lock(&mutex_);

  if (position_and_direction_.has_value()) {
    glUniform2f(uniform_runner_position_, position_and_direction_->p().x(),
                position_and_direction_->p().y());

    if (!guideline_.empty()) {
      // The shader currently only supports a max of 100 points (could be
      // increased if needed by modifying size of shader uniform array).
      const size_t kMaxGuidelineShaderPoints = 100;
      if (guideline_.size() > kMaxGuidelineShaderPoints) {
        guideline_.resize(kMaxGuidelineShaderPoints);
      }

      glUniform1i(uniform_line_point_count_, guideline_.size());
      std::vector<GLfloat> gl_points;
      for (const auto& point : guideline_) {
        gl_points.push_back(point.x());
        gl_points.push_back(point.y());
      }
      glUniform2fv(uniform_line_points_, gl_points.size(), gl_points.data());
    } else {
      glUniform1i(uniform_line_point_count_, 0);
    }

    if (!obstacles_.empty()) {
      // The shader currently only supports a max of 100 points (could be
      // increased if needed by modifying size of shader uniform array).
      const size_t kMaxObstacleShaderPoints = 100;
      if (obstacles_.size() > kMaxObstacleShaderPoints) {
        obstacles_.resize(kMaxObstacleShaderPoints);
      }

      glUniform1i(uniform_obstacle_point_count_, obstacles_.size());
      std::vector<GLfloat> obstacle_points;
      for (const auto& point : obstacles_) {
        obstacle_points.push_back(point.x());
        obstacle_points.push_back(point.y());
      }
      glUniform2fv(uniform_obstacle_points_, obstacle_points.size(),
                   obstacle_points.data());
    } else {
      glUniform1i(uniform_obstacle_point_count_, 0);
    }

    // Rotate the map so that it's aligned with the user's forward orientation.
    const Eigen::Vector2d yaw_pitch =
        util::ComputeYawPitch(position_and_direction_->q());
    Eigen::Affine3f t = Eigen::Affine3f::Identity();
    Eigen::Vector3f map_center(0.5, 0.5, 0.);
    t.pretranslate(-map_center);
    t.prerotate(Eigen::AngleAxisf(-yaw_pitch[0], Eigen::Vector3f::UnitZ()));
    t.pretranslate(map_center);

    transform_matrix = t.matrix();

    glUniform1f(
        uniform_rotation_movement_radians_,
        -control_signal_.rotation_movement_degrees * util::kDegreesToRadians);
    glUniform1f(uniform_lateral_movement_meters_,
                control_signal_.lateral_movement_meters);

    Eigen::Vector3d turn_point =
        control_signal_.turn_point.value_or(Eigen::Vector3d::Zero());
    glUniform2f(uniform_turn_point_, turn_point.x(), turn_point.y());
    glUniform1f(uniform_turn_angle_radians_,
                control_signal_.turn_angle_degrees * util::kDegreesToRadians);
    glUniform1i(uniform_stop_, control_signal_.stop ? 1 : 0);

    Eigen::Vector3d target_point =
        control_signal_.target_point.value_or(Eigen::Vector3d::Zero());
    glUniform2f(uniform_target_point_, target_point.x(), target_point.y());

    glUniform1f(uniform_target_rotation_movement_radians_,
                control_signal_.target_rotation_movement_degrees *
                    util::kDegreesToRadians);

    glUniform2f(uniform_runner_vector_, control_signal_.velocity_direction.x(),
                control_signal_.velocity_direction.y());

    glUniform1i(uniform_use_target_point_, 1);
  } else {
    glUniform2f(uniform_runner_position_, 0., 0.);
    glUniform2f(uniform_runner_vector_, 0., 0.);
    glUniform1i(uniform_line_point_count_, 0);
    glUniform1i(uniform_obstacle_point_count_, 0);
    glUniform1f(uniform_rotation_movement_radians_, 0.);
    glUniform1f(uniform_lateral_movement_meters_, 0.);
    glUniform2f(uniform_turn_point_, 0., 0.);
    glUniform1f(uniform_turn_angle_radians_, 0.);
    glUniform1i(uniform_stop_, 1);
  }

  glUniformMatrix4fv(uniform_matrix_, 1, false, transform_matrix.data());

  CheckGlError("bind uniforms");

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
  CheckGlError("bind texture");

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
  glUseProgram(0);

  glViewport(vp[0], vp[1], vp[2], vp[3]);

  CheckGlError("environment map renderer");
}

void EnvironmentMapRenderer::OnPose(
    const util::Transformation& position_and_direction) {
  absl::MutexLock lock(&mutex_);
  position_and_direction_ = position_and_direction;
}

void EnvironmentMapRenderer::OnControlSignal(
    const environment::ControlSignal& signal) {
  absl::MutexLock lock(&mutex_);
  control_signal_ = signal;
}

void EnvironmentMapRenderer::OnGuideline(
    const std::vector<Eigen::Vector3d>& guideline) {
  absl::MutexLock lock(&mutex_);
  guideline_ = guideline;
}

void EnvironmentMapRenderer::OnObstacles(
    const std::vector<Eigen::Vector2d>& obstacles) {
  absl::MutexLock lock(&mutex_);
  obstacles_ = obstacles;
}

void EnvironmentMapRenderer::Reset() {
  absl::MutexLock lock(&mutex_);
  control_signal_ = environment::ControlSignal();
  control_signal_.stop = true;
  position_and_direction_ = std::nullopt;
  guideline_.clear();
  obstacles_.clear();
}

}  // namespace guideline::visualization
