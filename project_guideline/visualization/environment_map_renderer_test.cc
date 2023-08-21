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

#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "Eigen/Core"
#include "project_guideline/environment/control_signal.h"
#include "project_guideline/testing/images.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/image_io.h"
#include "project_guideline/util/transformation.h"
#include "project_guideline/visualization/render_context.h"

namespace guideline::visualization {
namespace {

inline constexpr int kWidth = 2280;
inline constexpr int kHeight = 1080;

const char kTestDataDir[] =
    "/project_guideline/project_guideline/visualization/testdata/";

std::string GetTestDataPath(const std::string& filename) {
  return std::string(getenv("TEST_SRCDIR")) + kTestDataDir + filename;
}

TEST(EnvironmentMapRenderer, Render) {
  GL_ASSERT_OK_AND_ASSIGN(auto render_context,
                          RenderContext::Create(kWidth, kHeight));

  EnvironmentMapRenderer renderer;
  renderer.InitGL();
  renderer.SetViewport(kWidth, kHeight);

  util::Transformation pose =
      util::LookAt({-1, -2, 0}, {1, 2, 0}, Eigen::Vector3d::UnitZ());
  renderer.OnPose(pose);

  std::vector<Eigen::Vector3d> guideline = {
      {-4, -4, 0}, {-3, -3, 0}, {-2, -2, 0}, {-1, 0, 0}, {0, 1, 0},
      {0, 2, 0},   {1, 3, 0},   {1, 4, 0},   {2, 5, 0},  {2, 6, 0},
      {2, 8, 0},   {3, 9, 0},   {4, 10, 0},  {5, 12, 0}, {6, 14, 0}};
  renderer.OnGuideline(guideline);

  std::vector<Eigen::Vector2d> obstacles = {
      {-1, 0},  {-1, 1},  {-1, 2}, {-1, 1}, {-1, 3},
      {-2, -2}, {-3, -1}, {-4, 4}, {-5, 5}, {-6, 6}};
  renderer.OnObstacles(obstacles);

  environment::ControlSignal signal;
  signal.stop = false;
  signal.lateral_movement_meters = 3;
  signal.target_point = Eigen::Vector3d(2, 5, 0);
  signal.rotation_movement_degrees = -20;
  signal.target_rotation_movement_degrees = -20;
  signal.turn_point = Eigen::Vector3d(4, 10, 0);
  signal.turn_angle_degrees = 35;
  signal.velocity_direction = Eigen::Vector3d(1, 1, 0);
  renderer.OnControlSignal(signal);

  renderer.Render();

  util::Image rendered_image = render_context->GetImage();
  GL_ASSERT_OK_AND_ASSIGN(auto golden_image,
                          util::LoadImage(GetTestDataPath("map_golden.png")));
  EXPECT_TRUE(ImagesEqual(rendered_image, golden_image));

  // Uncomment to update golden image.
  // GL_ASSERT_OK(util::SaveImage(rendered_image, "/tmp/map_golden.png"));
}

TEST(EnvironmentMapRenderer, Stop) {
  GL_ASSERT_OK_AND_ASSIGN(auto render_context,
                          RenderContext::Create(kWidth, kHeight));

  EnvironmentMapRenderer renderer;
  renderer.InitGL();
  renderer.SetViewport(kWidth, kHeight);

  util::Transformation pose =
      util::LookAt({0, 0, 0}, {-1, -1, 0}, Eigen::Vector3d::UnitZ());
  renderer.OnPose(pose);

  environment::ControlSignal signal;
  signal.stop = true;
  renderer.OnControlSignal(signal);

  renderer.Render();

  util::Image rendered_image = render_context->GetImage();
  GL_ASSERT_OK_AND_ASSIGN(
      auto golden_image,
      util::LoadImage(GetTestDataPath("map_stop_golden.png")));
  EXPECT_TRUE(ImagesEqual(rendered_image, golden_image));

  // Uncomment to update golden image.
  // GL_ASSERT_OK(util::SaveImage(rendered_image, "/tmp/map_stop_golden.png"));
}

TEST(EnvironmentMapRenderer, StopWithLineAndObstacles) {
  GL_ASSERT_OK_AND_ASSIGN(auto render_context,
                          RenderContext::Create(kWidth, kHeight));

  EnvironmentMapRenderer renderer;
  renderer.InitGL();
  renderer.SetViewport(kWidth, kHeight);

  util::Transformation pose =
      util::LookAt({0, 0, 0}, {-1, -1, 0}, Eigen::Vector3d::UnitZ());
  renderer.OnPose(pose);

  std::vector<Eigen::Vector3d> guideline = {
      {-4, -4, 0}, {-3, -3, 0}, {-2, -2, 0}, {-1, 0, 0}, {0, 1, 0},
      {0, 2, 0},   {1, 3, 0},   {1, 4, 0},   {2, 5, 0},  {2, 6, 0},
      {2, 8, 0},   {3, 9, 0},   {4, 10, 0},  {5, 12, 0}, {6, 14, 0}};
  renderer.OnGuideline(guideline);

  std::vector<Eigen::Vector2d> obstacles = {
      {-1, 0},  {-1, 1},  {-1, 2}, {-1, 1}, {-1, 3},
      {-2, -2}, {-3, -1}, {-4, 4}, {-5, 5}, {-6, 6}};
  renderer.OnObstacles(obstacles);

  environment::ControlSignal signal;
  signal.stop = true;
  renderer.OnControlSignal(signal);

  renderer.Render();

  util::Image rendered_image = render_context->GetImage();
  GL_ASSERT_OK_AND_ASSIGN(
      auto golden_image,
      util::LoadImage(GetTestDataPath("map_stop_guideline_golden.png")));
  EXPECT_TRUE(ImagesEqual(rendered_image, golden_image));

  // Uncomment to update golden image.
  // GL_ASSERT_OK(util::SaveImage(rendered_image,
  //     "/tmp/map_stop_guideline_golden.png"));
}

TEST(EnvironmentMapRenderer, NoPose) {
  GL_ASSERT_OK_AND_ASSIGN(auto render_context,
                          RenderContext::Create(kWidth, kHeight));

  EnvironmentMapRenderer renderer;
  renderer.InitGL();
  renderer.SetViewport(kWidth, kHeight);

  renderer.Render();

  util::Image rendered_image = render_context->GetImage();
  GL_ASSERT_OK_AND_ASSIGN(
      auto golden_image,
      util::LoadImage(GetTestDataPath("map_nopose_golden.png")));
  EXPECT_TRUE(ImagesEqual(rendered_image, golden_image));

  // Uncomment to update golden image.
  // GL_ASSERT_OK(util::SaveImage(rendered_image,
  // "/tmp/map_nopose_golden.png"));
}

}  // namespace
}  // namespace guideline::visualization
