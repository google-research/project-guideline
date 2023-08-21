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

#include "project_guideline/environment/path_planning.h"

#include <cmath>
#include <csignal>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/transformation.h"

namespace guideline::environment {

namespace {

using ::Eigen::Vector3d;
using ::Eigen::Vector4d;
using util::Axis3;
using util::Transformation;

void assertNear(const Vector3d& a, const Vector3d& b, float tolerance) {
  ASSERT_NEAR(a[0], b[0], tolerance);
  ASSERT_NEAR(a[1], b[1], tolerance);
  ASSERT_NEAR(a[2], b[2], tolerance);
}

SimpleControlSystemOptions GetDefaultSimpleControlSystemOptions() {
  return SimpleControlSystemOptions();
}

TEST(SimpleControlSystemTest, TestComputeLateralMovement) {
  auto main_options = ControlSystemOptions();
  auto options = main_options.mutable_simple_control_system_options();
  *options = GetDefaultSimpleControlSystemOptions();
  auto status_or_control_system = SimpleControlSystem::Create(main_options);
  CHECK_OK(status_or_control_system);
  auto control_system = std::move(status_or_control_system.value());
  SimpleControlSystem* derived_ptr =
      dynamic_cast<SimpleControlSystem*>(control_system.get());

  std::vector<Vector3d> guideline_points;
  for (int i = -10; i < 11; ++i) {
    guideline_points.push_back(Vector3d(i, i, i));
  }
  Vector3d h1 = Vector3d(-1, 0, 1);
  Vector3d h2 = Vector3d(1, 0, -1);

  auto output =
      derived_ptr->ComputeLateralMovement(h1, guideline_points, Axis3::kX);
  ASSERT_FLOAT_EQ(output.lateral_movement_meters, std::sqrt(2) / 2);

  output = derived_ptr->ComputeLateralMovement(h1, guideline_points, Axis3::kY);
  ASSERT_FLOAT_EQ(output.lateral_movement_meters, -std::sqrt(2));

  output = derived_ptr->ComputeLateralMovement(h1, guideline_points, Axis3::kZ);
  ASSERT_FLOAT_EQ(output.lateral_movement_meters, std::sqrt(2) / 2);

  output = derived_ptr->ComputeLateralMovement(h2, guideline_points, Axis3::kY);
  ASSERT_FLOAT_EQ(output.lateral_movement_meters, std::sqrt(2));

  guideline_points.clear();
  for (int i = -10; i < -2; ++i) {
    guideline_points.push_back(Vector3d(i, i, i));
  }

  output = derived_ptr->ComputeLateralMovement(h1, guideline_points, Axis3::kX);
  ASSERT_TRUE(std::isnan(output.lateral_movement_meters));
}

TEST(SimpleControlSystemTest, TestComputeRotationalMovementLine) {
  std::vector<Vector3d> guideline_points;
  for (float i = 0; i < 18; i += 0.35) {
    guideline_points.push_back(Vector3d(i, i, i));
  }
  Vector3d human_direction = Vector3d(1, 0, 1);
  human_direction.normalize();

  auto main_options = ControlSystemOptions();
  auto options = main_options.mutable_simple_control_system_options();
  *options = GetDefaultSimpleControlSystemOptions();
  options->set_num_guideline_points_per_meter(2);
  options->set_distance_per_guideline_segment_meters(1);
  options->set_max_num_guideline_segments_to_use(3);
  auto status_or_control_system = SimpleControlSystem::Create(main_options);
  CHECK_OK(status_or_control_system);
  auto control_system = std::move(status_or_control_system.value());
  SimpleControlSystem* derived_ptr =
      dynamic_cast<SimpleControlSystem*>(control_system.get());

  // Failure case.
  auto output = derived_ptr->ComputeRotationalMovement(
      human_direction, guideline_points,
      /*closest_guideline_point_indx*/ 50, Axis3::kX);
  ASSERT_FLOAT_EQ(output.rotation_movement_degrees, 0);
  ASSERT_EQ(output.all_rotations.size(), 0);
  ASSERT_EQ(output.num_guideline_points_per_segment, 0);

  *options = GetDefaultSimpleControlSystemOptions();
  options->set_num_guideline_points_per_meter(2);
  options->set_distance_per_guideline_segment_meters(2);
  options->set_max_num_guideline_segments_to_use(3);
  auto status_or_control_system1 = SimpleControlSystem::Create(main_options);
  CHECK_OK(status_or_control_system1);
  auto control_system1 = std::move(status_or_control_system1.value());
  derived_ptr = dynamic_cast<SimpleControlSystem*>(control_system1.get());
  output = derived_ptr->ComputeRotationalMovement(
      human_direction, guideline_points,
      /*closest_guideline_point_indx*/ 0, Axis3::kX);
  ASSERT_FLOAT_EQ(output.rotation_movement_degrees, -45);
  ASSERT_EQ(output.all_rotations.size(), 12);
  ASSERT_EQ(output.num_guideline_points_per_segment, 4);
  output = derived_ptr->ComputeRotationalMovement(
      human_direction, guideline_points,
      /*closest_guideline_point_indx*/ 0, Axis3::kY);
  ASSERT_FLOAT_EQ(output.rotation_movement_degrees, 0);
  ASSERT_EQ(output.all_rotations.size(), 12);
  ASSERT_EQ(output.num_guideline_points_per_segment, 4);
  output = derived_ptr->ComputeRotationalMovement(
      human_direction, guideline_points,
      /*closest_guideline_point_indx*/ 0, Axis3::kZ);
  ASSERT_FLOAT_EQ(output.rotation_movement_degrees, 45);
  ASSERT_EQ(output.all_rotations.size(), 12);
  ASSERT_EQ(output.num_guideline_points_per_segment, 4);
  human_direction = Vector3d(3, 1, 1);
  human_direction.normalize();
  output = derived_ptr->ComputeRotationalMovement(
      human_direction, guideline_points,
      /*closest_guideline_point_indx*/ 0, Axis3::kX);
  ASSERT_FLOAT_EQ(output.rotation_movement_degrees, 0);
  ASSERT_EQ(output.all_rotations.size(), 12);
  ASSERT_EQ(output.num_guideline_points_per_segment, 4);
  output = derived_ptr->ComputeRotationalMovement(
      human_direction, guideline_points,
      /*closest_guideline_point_indx*/ 0, Axis3::kY);
  ASSERT_FLOAT_EQ(output.rotation_movement_degrees, -26.565052);
  ASSERT_EQ(output.all_rotations.size(), 12);
  ASSERT_EQ(output.num_guideline_points_per_segment, 4);
  output = derived_ptr->ComputeRotationalMovement(
      human_direction, guideline_points,
      /*closest_guideline_point_indx*/ 0, Axis3::kZ);
  ASSERT_FLOAT_EQ(output.rotation_movement_degrees, 26.565052);
  ASSERT_EQ(output.all_rotations.size(), 12);
  ASSERT_EQ(output.num_guideline_points_per_segment, 4);

  human_direction = Vector3d(1, 0, 1);
  human_direction.normalize();
  *options = GetDefaultSimpleControlSystemOptions();
  options->set_num_guideline_points_per_meter(2);
  options->set_distance_per_guideline_segment_meters(2);
  options->set_max_num_guideline_segments_to_use(15);
  auto status_or_control_system2 = SimpleControlSystem::Create(main_options);
  CHECK_OK(status_or_control_system2);
  auto control_system2 = std::move(status_or_control_system2.value());
  derived_ptr = dynamic_cast<SimpleControlSystem*>(control_system2.get());
  output = derived_ptr->ComputeRotationalMovement(
      human_direction, guideline_points,
      /*closest_guideline_point_indx*/ 0, Axis3::kX);
  ASSERT_FLOAT_EQ(output.rotation_movement_degrees, -45);
  ASSERT_EQ(output.all_rotations.size(), 12);
  ASSERT_EQ(output.num_guideline_points_per_segment, 4);
}

TEST(SimpleControlSystemTest, TestComputeRotationalMovementCurve) {
  std::vector<Vector3d> guideline_points;
  std::vector<float> xs = {0,   0.5, 1,   1.5, 2,   2.5, 3,   3.5, 4,
                           4.5, 5,   5.5, 6,   6.5, 7,   7.5, 8};
  std::vector<float> ys = {0,   0.5, 1,   1.5, 2,   2.5, 3,   3.5, 4,
                           4.5, 5,   5.5, 6,   6.5, 7,   7.5, 8};
  std::vector<float> zs = {0, 0, 0,  0, 0,   0, 0,   0, 0,
                           0, 0, .5, 1, 1.5, 2, 2.5, 3};

  for (int i = 0; i < xs.size(); ++i) {
    guideline_points.push_back(Vector3d(xs.at(i), ys.at(i), zs.at(i)));
  }
  Vector3d human_direction = Vector3d(1, 2, 3);
  human_direction.normalize();

  auto main_options = ControlSystemOptions();
  auto options = main_options.mutable_simple_control_system_options();
  *options = GetDefaultSimpleControlSystemOptions();
  options->set_num_guideline_points_per_meter(2);
  options->set_distance_per_guideline_segment_meters(2);
  options->set_max_num_guideline_segments_to_use(3);
  auto status_or_control_system = SimpleControlSystem::Create(main_options);
  CHECK_OK(status_or_control_system);
  auto control_system = std::move(status_or_control_system.value());
  SimpleControlSystem* derived_ptr =
      dynamic_cast<SimpleControlSystem*>(control_system.get());

  auto output = derived_ptr->ComputeRotationalMovement(
      human_direction, guideline_points,
      /*closest_guideline_point_indx*/ 5, Axis3::kX);
  ASSERT_FLOAT_EQ(output.rotation_movement_degrees, -37.874985);
  ASSERT_EQ(output.all_rotations.size(), 2);
  ASSERT_EQ(output.num_guideline_points_per_segment, 4);

  output = derived_ptr->ComputeRotationalMovement(
      human_direction, guideline_points,
      /*closest_guideline_point_indx*/ 5, Axis3::kY);
  ASSERT_FLOAT_EQ(output.rotation_movement_degrees, 53.1301);
  ASSERT_EQ(output.all_rotations.size(), 2);
  ASSERT_EQ(output.num_guideline_points_per_segment, 4);

  output = derived_ptr->ComputeRotationalMovement(
      human_direction, guideline_points,
      /*closest_guideline_point_indx*/ 5, Axis3::kZ);
  ASSERT_FLOAT_EQ(output.rotation_movement_degrees, -18.434948);
  ASSERT_EQ(output.all_rotations.size(), 2);
  ASSERT_EQ(output.num_guideline_points_per_segment, 4);
}

TEST(SimpleControlSystemTest, TestFindTurnPoint) {
  std::vector<Vector3d> guideline_points;
  std::vector<float> xs = {0,   0.5, 1,   1.5,  2,    2.5, 3,    3.5, 4,
                           4.5, 5,   5.5, 6,    6.5,  7,   7.5,  8,   8.5,
                           9.,  9.5, 10., 10.5, 11.5, 12,  12.5, 13};
  std::vector<float> ys = {0,   0.5, 1,   1.5, 2,   2.5, 3,   3.5, 4,
                           4.5, 5,   5.5, 6,   6.5, 7,   7.5, 8,   8,
                           8,   8,   8,   8,   8,   8,   8,   8};
  std::vector<float> zs = {0,   0, 0,   0, 0, 0, 0,  0,  0,  0,  0,  .5, 1,
                           1.5, 2, 2.5, 3, 4, 5, 14, 15, 16, 17, 18, 19, 20};
  for (int i = 0; i < xs.size(); ++i) {
    guideline_points.push_back(Vector3d(xs.at(i), ys.at(i), zs.at(i)));
  }
  Vector3d human_direction = Vector3d(1, 0, 0);
  auto main_options = ControlSystemOptions();
  auto options = main_options.mutable_simple_control_system_options();
  *options = GetDefaultSimpleControlSystemOptions();
  options->set_num_guideline_points_per_meter(2);
  options->set_distance_per_guideline_segment_meters(2);
  options->set_max_num_guideline_segments_to_use(7);
  auto status_or_control_system = SimpleControlSystem::Create(main_options);
  CHECK_OK(status_or_control_system);
  auto control_system = std::move(status_or_control_system.value());
  SimpleControlSystem* derived_ptr =
      dynamic_cast<SimpleControlSystem*>(control_system.get());
  auto input = derived_ptr->ComputeRotationalMovement(
      human_direction, guideline_points,
      /*closest_guideline_point_indx*/ 0, Axis3::kY);
  auto output = derived_ptr->FindTurnPoint(
      Vector3d(0, 0, 0), guideline_points, 0,
      input.num_guideline_points_per_segment, input.all_rotations,
      input.rotation_movement_degrees, Axis3::kY);
  ASSERT_EQ(output.turn_point, Vector3d(10, 8, 15));
  ASSERT_NEAR(output.turn_angle_degrees, -22.978, 0.001);

  auto output_too_close = derived_ptr->FindTurnPoint(
      Vector3d(8, 0, 0), guideline_points, 0,
      input.num_guideline_points_per_segment, input.all_rotations,
      input.rotation_movement_degrees, Axis3::kY);
  ASSERT_EQ(output_too_close.turn_point, Vector3d(10, 8, 15));
  ASSERT_FLOAT_EQ(output_too_close.turn_angle_degrees, -22.978394);

  options->set_turn_angle_threshold_degrees(16);
  auto status_or_control_system1 = SimpleControlSystem::Create(main_options);
  CHECK_OK(status_or_control_system1);
  auto control_system1 = std::move(status_or_control_system1.value());
  derived_ptr = dynamic_cast<SimpleControlSystem*>(control_system1.get());
  input = derived_ptr->ComputeRotationalMovement(
      human_direction, guideline_points,
      /*closest_guideline_point_indx*/ 0, Axis3::kZ);
  output = derived_ptr->FindTurnPoint(
      Vector3d(0, 0, 0), guideline_points, 0,
      input.num_guideline_points_per_segment, input.all_rotations,
      input.rotation_movement_degrees, Axis3::kZ);
  ASSERT_EQ(output.turn_point, std::nullopt);
  ASSERT_FLOAT_EQ(output.turn_angle_degrees, 0);
}

TEST(SimpleControlSystemTest, TestSimpleControlSystem_0) {
  std::vector<Obstacle> obstacles;
  std::vector<Vector3d> guideline_points;
  for (int i = 0; i < 10; ++i) {
    guideline_points.emplace_back(Vector3d(i, i, i));
  }
  auto main_options = ControlSystemOptions();
  auto options = main_options.mutable_simple_control_system_options();
  *options = GetDefaultSimpleControlSystemOptions();
  options->set_num_guideline_points_per_meter(2);
  options->set_distance_per_guideline_segment_meters(2);
  options->set_max_num_guideline_segments_to_use(5);
  options->set_lateral_movement_abs_min_threshold_meters(0.5);
  auto status_or_control_system = SimpleControlSystem::Create(main_options);
  CHECK_OK(status_or_control_system);
  auto control_system = std::move(status_or_control_system.value());
  auto human_position_direction = Transformation({1, 0, 0, 0}, {0, 0, -1});
  Vector3d human_velocity(0, 0, 0);
  auto control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kY);
  ASSERT_FLOAT_EQ(control_signals.lateral_movement_meters, 0);
  ASSERT_FLOAT_EQ(control_signals.rotation_movement_degrees, 45);
  ASSERT_EQ(control_signals.turn_point, std::nullopt);
  ASSERT_FLOAT_EQ(control_signals.turn_angle_degrees, 0);

  human_position_direction =
      Transformation({0, -std::sqrt(0.5), -std::sqrt(0.5), 0}, {1, 0, -1});
  control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kY);
  ASSERT_FLOAT_EQ(
      control_signals.lateral_movement_meters,
      std::sqrt(2) * std::abs(std::cos(util::DegreesToRadians(-135))));
  ASSERT_FLOAT_EQ(control_signals.rotation_movement_degrees, -135);
  ASSERT_EQ(control_signals.turn_point, std::nullopt);
  ASSERT_FLOAT_EQ(control_signals.turn_angle_degrees, 0);

  human_position_direction =
      Transformation({0, -std::sqrt(0.5), 0, std::sqrt(0.5)}, {1, 0, -1});
  control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kY);
  ASSERT_FLOAT_EQ(
      control_signals.lateral_movement_meters,
      std::sqrt(2) * std::abs(std::cos(util::DegreesToRadians(135))));
  ASSERT_FLOAT_EQ(control_signals.rotation_movement_degrees, 135);
  ASSERT_EQ(control_signals.turn_point, std::nullopt);
  ASSERT_FLOAT_EQ(control_signals.turn_angle_degrees, 0);

  human_position_direction =
      Transformation({0, -std::sqrt(0.5), 0, std::sqrt(0.5)}, {-1, 0, -1});
  control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kY);
  ASSERT_FLOAT_EQ(control_signals.lateral_movement_meters, 0);
  ASSERT_FLOAT_EQ(control_signals.rotation_movement_degrees, 135);
  ASSERT_EQ(control_signals.turn_point, std::nullopt);
  ASSERT_FLOAT_EQ(control_signals.turn_angle_degrees, 0);
}

TEST(SimpleControlSystemTest, TestSimpleControlSystem_1) {
  std::vector<Obstacle> obstacles;
  std::vector<Vector3d> guideline_points;
  std::vector<float> xs = {0,   0.5, 1,   1.5,  2,    2.5, 3,    3.5, 4,
                           4.5, 5,   5.5, 6,    6.5,  7,   7.5,  8,   8.5,
                           9.,  9.5, 10., 10.5, 11.5, 12,  12.5, 13};
  std::vector<float> ys = {0,   0.5, 1,   1.5, 2,   2.5, 3,   3.5, 4,
                           4.5, 5,   5.5, 6,   6.5, 7,   7.5, 8,   8,
                           8,   8,   8,   8,   8,   8,   8,   8};
  std::vector<float> zs = {0,   0, 0,   0, 0, 0, 0,  0,  0,  0,  0,  .5, 1,
                           1.5, 2, 2.5, 3, 4, 5, 14, 15, 16, 17, 18, 19, 20};
  for (int i = 0; i < xs.size(); ++i) {
    guideline_points.push_back(Vector3d(xs.at(i), ys.at(i), zs.at(i)));
  }
  auto main_options = ControlSystemOptions();
  auto options = main_options.mutable_simple_control_system_options();
  *options = GetDefaultSimpleControlSystemOptions();
  options->set_num_guideline_points_per_meter(2);
  options->set_distance_per_guideline_segment_meters(2);
  options->set_max_num_guideline_segments_to_use(3);
  auto status_or_control_system = SimpleControlSystem::Create(main_options);
  CHECK_OK(status_or_control_system);
  auto control_system = std::move(status_or_control_system.value());
  auto human_position_direction =
      Transformation({0, -std::sqrt(0.5), -0, -std::sqrt(0.5)}, {1, 0, -1});
  Vector3d human_velocity(0, 0, 0);
  auto control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kY);
  ASSERT_NEAR(control_signals.lateral_movement_meters, 0.9, 0.1);
  ASSERT_NEAR(control_signals.rotation_movement_degrees, -15, 1e-03);
  ASSERT_EQ(control_signals.turn_point, Vector3d(3, 3, 0));
  ASSERT_NEAR(control_signals.turn_angle_degrees, 15, 0.01);

  human_position_direction =
      Transformation({0, -std::sqrt(0.5), 0, std::sqrt(0.5)}, {1, 0, -1});
  control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kZ);
  ASSERT_FLOAT_EQ(control_signals.lateral_movement_meters, -0.5);
  ASSERT_NEAR(control_signals.rotation_movement_degrees, -135, 1e-03);
  ASSERT_EQ(control_signals.turn_point, std::nullopt);
  ASSERT_FLOAT_EQ(control_signals.turn_angle_degrees, 0);
}

TEST(SimpleControlSystemTest, TestTargetPoint) {
  std::vector<Obstacle> obstacles;
  std::vector<Vector3d> guideline_points;
  std::vector<float> xs = {0,   0.5, 1,   1.5,  2,    2.5, 3,    3.5, 4,
                           4.5, 5,   5.5, 6,    6.5,  7,   7.5,  8,   8.5,
                           9.,  9.5, 10., 10.5, 11.5, 12,  12.5, 13};
  std::vector<float> ys = {0,   0.5, 1,   1.5, 2,   2.5, 3,   3.5, 4,
                           4.5, 5,   5.5, 6,   6.5, 7,   7.5, 8,   8,
                           8,   8,   8,   8,   8,   8,   8,   8};
  std::vector<float> zs = {0,   0, 0,   0, 0, 0, 0,  0,  0,  0,  0,  .5, 1,
                           1.5, 2, 2.5, 3, 4, 5, 14, 15, 16, 17, 18, 19, 20};
  for (int i = 0; i < xs.size(); ++i) {
    guideline_points.push_back(Vector3d(xs.at(i), ys.at(i), zs.at(i)));
  }
  auto main_options = ControlSystemOptions();
  auto options = main_options.mutable_simple_control_system_options();
  *options = GetDefaultSimpleControlSystemOptions();
  options->set_num_guideline_points_per_meter(2);
  options->set_distance_per_guideline_segment_meters(2);
  options->set_max_num_guideline_segments_to_use(3);
  options->set_track_width_meters(4 * std::sqrt(0.5));
  auto status_or_control_system = SimpleControlSystem::Create(main_options);
  CHECK_OK(status_or_control_system);
  auto control_system = std::move(status_or_control_system.value());
  auto human_position_direction =
      Transformation({0, -std::sqrt(0.5), -0, -std::sqrt(0.5)}, {0, 0, 0});
  Vector3d human_velocity(2, 0, 0);

  auto control_signal = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kZ);
  ASSERT_EQ(control_signal.target_point, Vector3d(4, 4, 0));
  ASSERT_NEAR(control_signal.target_rotation_movement_degrees, -45, 0.01);
  ASSERT_EQ(control_signal.guideline_follow_point, Vector3d(4, 4, 0));
  assertNear(control_signal.right_edge_point.value(), Vector3d(2, -2, 0),
             0.00001f);
  assertNear(control_signal.left_edge_point.value(), Vector3d(-2, 2, 0),
             0.00001f);

  human_velocity = Vector3d(3, 3, 0);
  control_signal = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kZ);
  ASSERT_EQ(control_signal.target_point, Vector3d(7, 7, 2));
  ASSERT_NEAR(control_signal.target_rotation_movement_degrees, 0, 0.01);

  human_velocity = Vector3d(1, -1, 0);
  control_signal = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kZ);
  ASSERT_EQ(control_signal.target_point, Vector3d(3, 3, 0));
  ASSERT_NEAR(control_signal.target_rotation_movement_degrees, -90, 0.01);

  human_velocity = Vector3d(1, 4, 0);
  control_signal = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kZ);
  ASSERT_EQ(control_signal.target_point, Vector3d(6.5, 6.5, 1.5));
  ASSERT_NEAR(control_signal.target_rotation_movement_degrees, 31., 0.1);
}

TEST(SimpleControlSystemTest, TestTriggerStopSignal) {
  std::vector<Obstacle> obstacles;
  std::vector<Vector3d> guideline_points;
  for (int i = 0; i < 10; ++i) {
    guideline_points.emplace_back(Vector3d(i, i, i));
  }
  auto main_options = ControlSystemOptions();
  auto options = main_options.mutable_simple_control_system_options();
  *options = GetDefaultSimpleControlSystemOptions();
  options->set_num_guideline_points_per_meter(2);
  options->set_distance_per_guideline_segment_meters(2);
  options->set_max_num_guideline_segments_to_use(3);
  auto status_or_control_system = SimpleControlSystem::Create(main_options);
  CHECK_OK(status_or_control_system);
  auto control_system = std::move(status_or_control_system.value());

  // Human past end of line, so control signal is initially stop.
  auto human_position_direction =
      Transformation({0, -std::sqrt(0.5), -0, -std::sqrt(0.5)}, {15, 15, 15});
  Vector3d human_velocity(0, 0, 0);
  auto control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kY);
  ASSERT_TRUE(control_signals.stop);

  // Human in valid position near start of line.
  human_position_direction =
      Transformation({0, -std::sqrt(0.5), -0, -std::sqrt(0.5)}, {-2, 1, 0});
  control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kY);

  ASSERT_NEAR(control_signals.lateral_movement_meters, -1, 0.01);
  ASSERT_FALSE(control_signals.stop);
  ASSERT_EQ(control_signals.turn_point, std::nullopt);
  ASSERT_EQ(control_signals.turn_angle_degrees, 0);

  // Out-of-range human position for enough frames will trigger stop.
  human_position_direction =
      Transformation({0, -std::sqrt(0.5), -0, -std::sqrt(0.5)}, {-5, 1, 0});
  for (int i = 0; i < control_system->GetConfig()
                              .simple_control_system_options()
                              .lateral_stop_min_consecutive_frames() -
                          1;
       ++i) {
    control_signals = control_system->GenerateControlSignal(
        human_position_direction, human_velocity, guideline_points, obstacles,
        Axis3::kY);
    ASSERT_FALSE(control_signals.stop);
  }
  control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kY);
  ASSERT_TRUE(control_signals.stop);

  // Human in valid position near start of line resets the previous stop signal.
  human_position_direction =
      Transformation({0, -std::sqrt(0.5), -0, -std::sqrt(0.5)}, {-2, 1, 0});
  control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kY);
  ASSERT_FALSE(control_signals.stop);
}

TEST(SimpleControlSystemTest, ObstaclesPresent) {
  std::vector<Obstacle> obstacles;
  obstacles.emplace_back();
  std::vector<Vector3d> guideline_points;
  for (int i = 0; i < 10; ++i) {
    guideline_points.emplace_back(Vector3d(i, i, i));
  }

  auto main_options = ControlSystemOptions();
  auto options = main_options.mutable_simple_control_system_options();
  *options = GetDefaultSimpleControlSystemOptions();
  options->set_obstacle_smoothing_min_interval_frames(3);

  auto control_system = SimpleControlSystem::Create(main_options).value();

  // Human in valid position near start of line.
  auto human_position_direction =
      Transformation({0, -std::sqrt(0.5), -0, -std::sqrt(0.5)}, {-2, 1, 0});
  Vector3d human_velocity(0, 0, 0);

  // Obtacle present. Due to smoothing there will be 3 frames required before
  // present in control signal output.
  auto control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kY);
  ASSERT_FALSE(control_signals.obstacle_ahead);
  control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kY);
  ASSERT_FALSE(control_signals.obstacle_ahead);
  control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, obstacles,
      Axis3::kY);
  ASSERT_TRUE(control_signals.obstacle_ahead);

  // No obstacles present. Due to smoothing there will be 3 frames required
  // before absent in control signal output.
  control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, {},
      Axis3::kY);
  ASSERT_TRUE(control_signals.obstacle_ahead);
  control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, {},
      Axis3::kY);
  ASSERT_TRUE(control_signals.obstacle_ahead);
  control_signals = control_system->GenerateControlSignal(
      human_position_direction, human_velocity, guideline_points, {},
      Axis3::kY);
  ASSERT_FALSE(control_signals.obstacle_ahead);
}

}  // namespace
}  // namespace guideline::environment
