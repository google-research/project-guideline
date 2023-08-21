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

#include "project_guideline/util/hit_test_util.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "Eigen/Core"
#include "project_guideline/camera/cv_camera_model.h"
#include "project_guideline/testing/predicates.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/transformation.h"

namespace guideline::util {
namespace {

using ::Eigen::Vector2d;
using ::Eigen::Vector3d;
using ::Eigen::Vector4d;
using util::Transformation;

static const camera::CvCameraModel* kCameraModel =
    new camera::CvCameraModel(640, 480, {400., 400., 320.0, 240.0});

TEST(HitTestPlane, LookStraightDownCenterHit) {
  util::Transformation look_down = util::LookAt(
      /*position=*/Vector3d::Zero(),
      /*look_at=*/Vector3d::UnitZ() * -1,
      /*up=*/Vector3d::UnitZ());

  Transformation world_T_plane(Vector3d(100, 200, 300));
  Transformation world_T_camera(look_down.q(), Vector3d(100, 200, 310));

  auto status_or_result = HitTestPlane(Vector2d(320, 240), world_T_plane,
                                       world_T_camera, *kCameraModel);

  GL_ASSERT_OK_AND_ASSIGN(auto hit_result, status_or_result);
  ASSERT_FLOAT_EQ(10, hit_result.hit_distance);
  EXPECT_TRANSFORMATION_APPROX(hit_result.hit_pose, world_T_plane, 1e-8);
}

TEST(HitTestPlane, LookStraightDownCornerHit) {
  util::Transformation look_down = util::LookAt(
      /*camera_position=*/Vector3d::Zero(),
      /*look_at_point=*/Vector3d::UnitZ() * -1,
      /*up=*/Vector3d::UnitZ());

  Transformation world_T_plane(Vector3d(100, 200, 300));
  Transformation world_T_camera(look_down.q(), Vector3d(100, 200, 310));

  auto status_or_result = HitTestPlane(Vector2d(0, 0), world_T_plane,
                                       world_T_camera, *kCameraModel);

  GL_ASSERT_OK_AND_ASSIGN(auto hit_result, status_or_result);
  ASSERT_FLOAT_EQ(14.142136, hit_result.hit_distance);
  EXPECT_TRANSFORMATION_APPROX(
      hit_result.hit_pose,
      Transformation(world_T_plane.q(), Vector3d(92, 206, 300)), 1e-8);
}

TEST(HitTestPlane, LookAtPointCenterHit) {
  Vector3d camera_position(100, 200, 310);
  Vector3d plane_point(92, 206, 300);

  // This looks at the same point as result from previous test. Distance should
  // be the same and a center hit should be exactly this point.
  util::Transformation look_at = util::LookAt(camera_position, plane_point,
                                              /*up=*/Vector3d::UnitZ());

  Transformation world_T_plane(Vector3d(100, 200, 300));
  Transformation world_T_camera(look_at.q(), camera_position);

  auto status_or_result = HitTestPlane(Vector2d(320, 240), world_T_plane,
                                       world_T_camera, *kCameraModel);

  GL_ASSERT_OK_AND_ASSIGN(auto hit_result, status_or_result);
  ASSERT_FLOAT_EQ(14.142136, hit_result.hit_distance);
  EXPECT_TRANSFORMATION_APPROX(hit_result.hit_pose,
                               Transformation(world_T_plane.q(), plane_point),
                               1e-8);
}

TEST(HitTestPlane, ParallelToPlane) {
  util::Transformation look_forward = util::LookAt(
      /*camera_position=*/Vector3d::Zero(),
      /*look_at_point=*/Vector3d::UnitY(),
      /*up=*/Vector3d::UnitZ());

  Transformation world_T_plane(Vector3d(100, 200, 300));
  Transformation world_T_camera(look_forward.q(), Vector3d(100, 200, 310));

  auto status_or_result = HitTestPlane(Vector2d(320, 240), world_T_plane,
                                       world_T_camera, *kCameraModel);

  EXPECT_FALSE(status_or_result.ok());
}

TEST(HitTestPlane, NoIntersection) {
  util::Transformation look_forward = util::LookAt(
      /*camera_position=*/Vector3d::Zero(),
      /*look_at_point=*/Vector3d::UnitY(),
      /*up=*/Vector3d::UnitZ());

  Transformation world_T_plane(Vector3d(100, 200, 300));
  Transformation world_T_camera(look_forward.q(), Vector3d(100, 200, 310));

  auto status_or_result = HitTestPlane(Vector2d(320, 200), world_T_plane,
                                       world_T_camera, *kCameraModel);

  EXPECT_FALSE(status_or_result.ok());
}

TEST(HitTestPlane, NearHorizon) {
  util::Transformation look_forward = util::LookAt(
      /*camera_position=*/Vector3d::Zero(),
      /*look_at_point=*/Vector3d::UnitY(),
      /*up=*/Vector3d::UnitZ());

  Transformation world_T_plane(Vector3d(100, 200, 300));
  Transformation world_T_camera(look_forward.q(), Vector3d(100, 200, 310));

  auto status_or_result = HitTestPlane(Vector2d(320, 241), world_T_plane,
                                       world_T_camera, *kCameraModel);

  GL_ASSERT_OK_AND_ASSIGN(auto hit_result, status_or_result);
  ASSERT_FLOAT_EQ(4000.0125, hit_result.hit_distance);
  EXPECT_TRANSFORMATION_APPROX(
      hit_result.hit_pose,
      Transformation(world_T_plane.q(), Vector3d(100, 4200, 300)), 1e-8);
}

}  // namespace
}  // namespace guideline::util
