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

#include "project_guideline/environment/human_representation.h"

#include <cmath>
#include <utility>

#include "gtest/gtest.h"
#include "absl/time/time.h"
#include "Eigen/Core"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/testing/status_matchers.h"
#include <google/protobuf/util/time_util.h>

namespace guideline::environment {

namespace {

using ::Eigen::Vector3d;
using ::Eigen::Vector4d;
using util::Transformation;

TEST(HumanV0, NormalBehaviour) {
  auto options = HumanRepresentationOptions();
  options.mutable_camera_pose_based_human_options();
  auto status_or_human = CameraPoseBasedHuman::Create(options);
  GL_ASSERT_OK(status_or_human);
  auto human = std::move(status_or_human.value());

  for (int i = 0; i < 100; ++i) {
    human->UpdatePositionAndDirection(
        Transformation({1, 0, 0, 0}, Vector3d(i, i, i)), i);
  }
  ASSERT_EQ(human->NumEntries(), 100);
  ASSERT_EQ(human->CurrentPositionAndDirection()->q().coeffs(),
            Vector4d(0, 0, 0, 1));
  ASSERT_EQ(human->CurrentPositionAndDirection()->p(), Vector3d(99, 99, 99));

  human->UpdatePositionAndDirection(
      Transformation({1, -1, 0, 0}, Vector3d(100, 100, 100)), 100);
  ASSERT_EQ(human->NumEntries(), 101);
  ASSERT_NEAR(human->CurrentPositionAndDirection()->q().x(), -std::sqrt(2) / 2,
              1e-04);
  ASSERT_NEAR(human->CurrentPositionAndDirection()->q().w(), std::sqrt(2) / 2,
              1e-04);
  ASSERT_EQ(human->CurrentPositionAndDirection()->q().y(), 0);
  ASSERT_EQ(human->CurrentPositionAndDirection()->q().z(), 0);
  ASSERT_EQ(human->CurrentPositionAndDirection()->p(), Vector3d(100, 100, 100));

  auto clear_status = human->ClearEntries(5);
  GL_ASSERT_OK(clear_status);
  ASSERT_EQ(human->NumEntries(), 96);
}

TEST(HumanV0, Errors) {
  auto options = HumanRepresentationOptions();
  options.mutable_camera_pose_based_human_options();
  auto status_or_human = CameraPoseBasedHuman::Create(options);
  GL_ASSERT_OK(status_or_human);
  auto human = std::move(status_or_human.value());
  ASSERT_FALSE(human->CurrentPositionAndDirection().ok());
  ASSERT_FALSE(human->CurrentSpeed().ok());
  ASSERT_FALSE(human->ClearEntries(1).ok());
}

TEST(HumanV0, VelocityVector) {
  auto options = HumanRepresentationOptions();
  auto cp_options = options.mutable_camera_pose_based_human_options();
  *cp_options->mutable_velocity_window() =
      google::protobuf::util::TimeUtil::MillisecondsToDuration(500);
  auto status_or_human = CameraPoseBasedHuman::Create(options);
  GL_ASSERT_OK(status_or_human);
  auto human = std::move(status_or_human.value());

  human->UpdatePositionAndDirection(
      Transformation({1, 0, 0, 0}, Vector3d(0, 0, 0)),
      absl::ToInt64Microseconds(absl::Milliseconds(100)));

  human->UpdatePositionAndDirection(
      Transformation({1, 0, 0, 0}, Vector3d(10, 10, 10)),
      absl::ToInt64Microseconds(absl::Milliseconds(200)));

  ASSERT_FALSE(human->VelocityVector().ok());  // Not enough measurements.
  ASSERT_FALSE(human->CurrentSpeed().ok());

  human->UpdatePositionAndDirection(
      Transformation({1, 0, 0, 0}, Vector3d(15, 15, 15)),
      absl::ToInt64Microseconds(absl::Milliseconds(300)));

  human->UpdatePositionAndDirection(
      Transformation({1, 0, 0, 0}, Vector3d(25, 25, 25)),
      absl::ToInt64Microseconds(absl::Milliseconds(700)));

  auto velocity = human->VelocityVector();
  GL_ASSERT_OK(velocity);
  ASSERT_EQ(*velocity, Vector3d(30, 30, 30));  // (25m - 10m) / 500ms = 30m/s

  auto speed = human->CurrentSpeed();
  ASSERT_NEAR(*speed, 51.96, 0.01);  // sqrt(30^2 + 30^2 + 30^2) = 51.96

  human->UpdatePositionAndDirection(
      Transformation({1, 0, 0, 0}, Vector3d(-25, -25, -25)),
      absl::ToInt64Microseconds(absl::Milliseconds(1200)));

  velocity = human->VelocityVector();
  GL_ASSERT_OK(velocity);
  ASSERT_EQ(*velocity, Vector3d(-100, -100, -100));

  speed = human->CurrentSpeed();
  ASSERT_NEAR(*speed, 173.2, 0.01);

  human->UpdatePositionAndDirection(
      Transformation({1, 0, 0, 0}, Vector3d(-25, -25, -25)),
      absl::ToInt64Microseconds(absl::Milliseconds(1700)));

  velocity = human->VelocityVector();
  GL_ASSERT_OK(velocity);
  ASSERT_EQ(*velocity, Vector3d(0, 0, 0));

  speed = human->CurrentSpeed();
  ASSERT_NEAR(*speed, 0., 0.01);
}

}  // namespace
}  // namespace guideline::environment
