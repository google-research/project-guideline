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

#include "project_guideline/util/geometry.h"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "Eigen/Core"  // keep include
#include "project_guideline/testing/predicates.h"

namespace guideline::util {
namespace {

using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

constexpr double kDoubleTolerance = 1e-6;
constexpr double kQuaternionTolerance = 1e-3;

TEST(Geometry, JplToHamilton) {
  EXPECT_EIGEN_APPROX(JplToHamilton({0, 0, 0, 1}).coeffs(),
                      Vector4d(0, 0, 0, 1), kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(JplToHamilton({0, 0.707, 0.707, 0}).coeffs(),
                      Vector4d(0, -0.707, -0.707, 0), kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(JplToHamilton({-0.029, 0.908, 0.416, 0.023}).coeffs(),
                      Vector4d(0.029, -0.908, -0.416, 0.023),
                      kQuaternionTolerance);
}

TEST(Geometry, HamiltonToJpl) {
  EXPECT_EIGEN_APPROX(HamiltonToJpl({1, 0, 0, 0}), Vector4d(0, 0, 0, 1),
                      kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(HamiltonToJpl({0, 0, -0.707, -0.707}),
                      Vector4d(0, 0.707, 0.707, 0), kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(HamiltonToJpl({0.023, 0.029, -0.908, -0.416}),
                      Vector4d(-0.029, 0.908, 0.416, 0.023),
                      kQuaternionTolerance);
}

TEST(Geometry, ComputeDistance2D) {
  const Vector3d a(-1, -2, -3);
  const Vector3d b(4, 5, 6);
  EXPECT_DOUBLE_EQ(ComputeDistance2D(a, b, Axis3::kX),
                   std::sqrt(std::pow(7, 2) + std::pow(9, 2)));
  EXPECT_DOUBLE_EQ(ComputeDistance2D(a, b, Axis3::kY),
                   std::sqrt(std::pow(5, 2) + std::pow(9, 2)));
  EXPECT_DOUBLE_EQ(ComputeDistance2D(a, b, Axis3::kZ),
                   std::sqrt(std::pow(5, 2) + std::pow(7, 2)));

  EXPECT_DOUBLE_EQ(
      ComputeDistance2D(Vector3d::Zero(), Vector3d::Zero(), Axis3::kX), 0);
  EXPECT_DOUBLE_EQ(
      ComputeDistance2D(Vector3d::Ones(), Vector3d::Ones(), Axis3::kX), 0);

  EXPECT_EQ(ComputeDistance2D({std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::infinity()},
                              Vector3d::Zero(), Axis3::kX),
            std::numeric_limits<double>::infinity());
  EXPECT_TRUE(
      std::isnan(ComputeDistance2D({std::numeric_limits<double>::infinity(),
                                    std::numeric_limits<double>::infinity(),
                                    std::numeric_limits<double>::infinity()},
                                   {std::numeric_limits<double>::infinity(),
                                    std::numeric_limits<double>::infinity(),
                                    std::numeric_limits<double>::infinity()},
                                   Axis3::kX)));
}

TEST(Geometry, ComputeUnitDirection2D) {
  const Vector3d src = {-1, -2, -3};
  const Vector3d dst = {4, 5, 6};
  EXPECT_EIGEN_APPROX(ComputeUnitDirection2D(src, dst, Axis3::kX),
                      Vector3d(0, 0.6139406, 0.7893522), kDoubleTolerance);
  EXPECT_EIGEN_APPROX(ComputeUnitDirection2D(dst, src, Axis3::kX),
                      Vector3d(0, -0.6139406, -0.7893522), kDoubleTolerance);

  EXPECT_EIGEN_APPROX(ComputeUnitDirection2D(src, dst, Axis3::kY),
                      Vector3d(0.4856429, 0, 0.8741572), kDoubleTolerance);
  EXPECT_EIGEN_APPROX(ComputeUnitDirection2D(dst, src, Axis3::kY),
                      Vector3d(-0.4856429, 0, -0.8741572), kDoubleTolerance);

  EXPECT_EIGEN_APPROX(ComputeUnitDirection2D(src, dst, Axis3::kZ),
                      Vector3d(0.5812381, 0.8137334, 0), kDoubleTolerance);
  EXPECT_EIGEN_APPROX(ComputeUnitDirection2D(dst, src, Axis3::kZ),
                      Vector3d(-0.5812381, -0.8137334, 0), kDoubleTolerance);

  EXPECT_EIGEN_APPROX(
      ComputeUnitDirection2D(Vector3d::Zero(), Vector3d::Zero(), Axis3::kX),
      Vector3d::Zero(), kDoubleTolerance);

  auto direction =
      ComputeUnitDirection2D(Vector3d::Zero(),
                             {std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity()},
                             Axis3::kX);
  EXPECT_NEAR(direction.x(), 0, kDoubleTolerance);
  EXPECT_TRUE(std::isnan(direction.y()));
  EXPECT_TRUE(std::isnan(direction.z()));
}

TEST(Geometry, ComputeAngle2D) {
  EXPECT_DOUBLE_EQ(
      (ComputeAngle2D(Vector3d::Zero(), Vector3d::Zero(), Axis3::kX)), 0);

  Vector3d src = Vector3d(1, 0, 0);
  Vector3d dst = Vector3d(0, 1, 0);
  EXPECT_NEAR(ComputeAngle2D(src, dst, Axis3::kZ), DegreesToRadians(90),
              kDoubleTolerance);
  EXPECT_NEAR(ComputeAngle2D(dst, src, Axis3::kZ), DegreesToRadians(-90),
              kDoubleTolerance);

  src = Vector3d(1, 0, 0);
  dst = Vector3d(-1, 0.01, 0);
  EXPECT_NEAR(ComputeAngle2D(src, dst, Axis3::kZ), DegreesToRadians(179.42706),
              kDoubleTolerance);
  EXPECT_NEAR(ComputeAngle2D(dst, src, Axis3::kZ), DegreesToRadians(-179.42706),
              kDoubleTolerance);

  src = Vector3d(1, 1, 0);
  dst = Vector3d(1, 0, 0);
  EXPECT_NEAR(ComputeAngle2D(src, dst, Axis3::kZ), DegreesToRadians(-45),
              kDoubleTolerance);
  EXPECT_NEAR(ComputeAngle2D(dst, src, Axis3::kZ), DegreesToRadians(45),
              kDoubleTolerance);

  src = Vector3d(1, 2, 3);
  dst = Vector3d(1, 1, 1);
  EXPECT_NEAR(ComputeAngle2D(src, dst, Axis3::kX), DegreesToRadians(-11.309933),
              kDoubleTolerance);
  EXPECT_NEAR(ComputeAngle2D(src, dst, Axis3::kY), DegreesToRadians(26.565052),
              kDoubleTolerance);
  EXPECT_NEAR(ComputeAngle2D(src, dst, Axis3::kZ), DegreesToRadians(-18.434948),
              kDoubleTolerance);
  EXPECT_NEAR(ComputeAngle2D(dst, src, Axis3::kX), DegreesToRadians(11.309933),
              kDoubleTolerance);
  EXPECT_NEAR(ComputeAngle2D(dst, src, Axis3::kY), DegreesToRadians(-26.565052),
              kDoubleTolerance);
  EXPECT_NEAR(ComputeAngle2D(dst, src, Axis3::kZ), DegreesToRadians(18.434948),
              kDoubleTolerance);
}

TEST(Geometry, ComputeForwardDirectionFromQuaternion) {
  EXPECT_EIGEN_APPROX(ComputeForwardDirectionFromQuaternion({1, 0, 0, 0}),
                      Vector3d(0, 0, 1), kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(
      ComputeForwardDirectionFromQuaternion({0, -0, -0.707, -0.707}),
      Vector3d(0, 1, 0), kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(
      ComputeForwardDirectionFromQuaternion({0, 0.707, 0, 0.707}),
      Vector3d(1, 0, 0), kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(
      ComputeForwardDirectionFromQuaternion(
          {0.023433242838234745, 0.02943894758851117, -0.9081868762719156,
           -0.4168702786988126}),
      Vector3d(-0.06710797180000005, 0.7558125324286674, -0.6513401077353175),
      kQuaternionTolerance);
}

TEST(Geometry, ComputePerpendicularDirection2D) {
  EXPECT_EIGEN_APPROX(
      ComputePerpendicularDirection2D({0, 0, 0}, {0, 0, 1}, Axis3::kX),
      Vector3d(0, 1, 0));

  EXPECT_EIGEN_APPROX(
      ComputePerpendicularDirection2D({3, 4, 1}, {6, 8, 2}, Axis3::kZ),
      Vector3d(0.8, -0.6, 0));

  EXPECT_EIGEN_APPROX(
      ComputePerpendicularDirection2D({0, 0, 0}, {0, 0, 1}, Axis3::kY),
      Vector3d(-1, 0, 0));

  EXPECT_EIGEN_APPROX(
      ComputePerpendicularDirection2D({0, 0, 1}, {0, 0, 0}, Axis3::kY),
      Vector3d(1, 0, 0));

  EXPECT_EIGEN_APPROX(
      ComputePerpendicularDirection2D({0, 1, 0}, {5, 2, 12}, Axis3::kY),
      Vector3d(-0.9230769, 0, 0.3846153), kDoubleTolerance);
}

TEST(Geometry, ComputePerpendicularDistanceFromLine2D) {
  EXPECT_NEAR(ComputePerpendicularDistanceFromLine2D({0, 0, 0}, {0, 0, 0},
                                                     {1, 1, 1}, Axis3::kZ),
              0, kDoubleTolerance);
  EXPECT_NEAR(ComputePerpendicularDistanceFromLine2D({0, 0, 0}, {1, 1, 1},
                                                     {2, 2, 2}, Axis3::kZ),
              0, kDoubleTolerance);
  EXPECT_TRUE(std::isnan(ComputePerpendicularDistanceFromLine2D(
      {0, 0, 0}, {1, 1, 1}, {1, 1, 1}, Axis3::kZ)));

  EXPECT_NEAR(ComputePerpendicularDistanceFromLine2D({1, 0, 0}, {0, 4, 0},
                                                     {0, -4, 0}, Axis3::kZ),
              1, kDoubleTolerance);
  EXPECT_NEAR(ComputePerpendicularDistanceFromLine2D({-1, 0, 7}, {0, 4, 7},
                                                     {0, -4, 7}, Axis3::kZ),
              -1, kDoubleTolerance);

  EXPECT_NEAR(ComputePerpendicularDistanceFromLine2D({0, 1, 0}, {7, 0, 4},
                                                     {7, 0, -4}, Axis3::kX),
              1, kDoubleTolerance);
  EXPECT_NEAR(ComputePerpendicularDistanceFromLine2D({1, 7, 0}, {0, 7, 4},
                                                     {0, 7, -4}, Axis3::kY),
              -1, kDoubleTolerance);

  EXPECT_NEAR(ComputePerpendicularDistanceFromLine2D({1, 2, 3}, {4, 5, 6},
                                                     {2, 7, 9}, Axis3::kZ),
              4.2426406, kDoubleTolerance);
  EXPECT_NEAR(ComputePerpendicularDistanceFromLine2D({-1, -2, -3}, {-2, 5, 6},
                                                     {3, 1, 7}, Axis3::kZ),
              -4.8413866, kDoubleTolerance);
}

TEST(Geometry, LookAt) {
  auto t = LookAt(Vector3d::Zero(), {0, 0, 1}, Vector3d::UnitZ());
  EXPECT_EIGEN_APPROX(t.q().coeffs(), Vector4d(0, 0, 0, 1));
  EXPECT_EIGEN_APPROX(t.p(), Vector3d::Zero());

  t = LookAt({4, 5, 6}, {4, 5, 10}, Vector3d::UnitZ());
  EXPECT_EIGEN_APPROX(t.q().coeffs(), Vector4d(0, 0, 0, 1));
  EXPECT_EIGEN_APPROX(t.p(), Vector3d(4, 5, 6));

  t = LookAt({-2, 2, 1}, {5, 1, -3}, Vector3d::UnitZ());
  EXPECT_EIGEN_APPROX(t.q().coeffs(), Vector4d(-0.566, 0.6526, -0.3806, 0.3301),
                      kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(t.p(), Vector3d(-2, 2, 1));

  t = LookAt({1, 2, 3}, {-5, -7, -9}, Vector3d::UnitZ());
  EXPECT_EIGEN_APPROX(t.q().coeffs(), Vector4d(0.2705, 0.8934, -0.3432, -0.104),
                      kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(t.p(), Vector3d(1, 2, 3));

  t = LookAt({1, 2, 3}, {-5, -7, -9}, Vector3d::UnitX());
  EXPECT_EIGEN_APPROX(t.q().coeffs(), Vector4d(-0.6164, 0.701, -0.093, -0.346),
                      kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(t.p(), Vector3d(1, 2, 3));

  t = LookAt({1, 2, 3}, {-5, -7, -9}, Vector3d::UnitY());
  EXPECT_EIGEN_APPROX(t.q().coeffs(), Vector4d(0.931, 0.067, -0.22, 0.283),
                      kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(t.p(), Vector3d(1, 2, 3));

  t = LookAt({1, 2, 3}, {1, 2, 3}, Vector3d::UnitZ());
  EXPECT_EIGEN_APPROX(t.q().coeffs(), Vector4d(1, 0, 0, 0),
                      kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(t.p(), Vector3d(1, 2, 3));

  t = LookAt({1, 2, 3}, {1, 2, 3}, Vector3d::UnitX());
  EXPECT_EIGEN_APPROX(t.q().coeffs(), Vector4d(1, 0, 0, 0),
                      kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(t.p(), Vector3d(1, 2, 3));

  t = LookAt({1, 2, 3}, {1, 2, 3}, Vector3d::UnitY());
  EXPECT_EIGEN_APPROX(t.q().coeffs(), Vector4d(1, 0, 0, 0),
                      kQuaternionTolerance);
  EXPECT_EIGEN_APPROX(t.p(), Vector3d(1, 2, 3));
}

TEST(Geometry, ComputeYawPitch) {
  Transformation t = util::LookAt({0, 0, 0}, {0, 1, 0}, Vector3d::UnitZ());
  Eigen::Vector2d yp = ComputeYawPitch(t.q()) / kDegreesToRadians;
  EXPECT_EIGEN_APPROX(yp, Vector2d(0, 0));

  t = util::LookAt({0, 0, 0}, {1, 1, 0}, Vector3d::UnitZ());
  yp = ComputeYawPitch(t.q()) / kDegreesToRadians;
  EXPECT_EIGEN_APPROX(yp, Vector2d(45, 0));

  t = util::LookAt({0, 0, 0}, {-1, 1, 0}, Vector3d::UnitZ());
  yp = ComputeYawPitch(t.q()) / kDegreesToRadians;
  EXPECT_EIGEN_APPROX(yp, Vector2d(-45, 0));

  t = util::LookAt({0, 0, 0}, {-1, -1, 0}, Vector3d::UnitZ());
  yp = ComputeYawPitch(t.q()) / kDegreesToRadians;
  EXPECT_EIGEN_APPROX(yp, Vector2d(-135, 0));

  t = util::LookAt({0, 0, 0}, {1, -1, 0}, Vector3d::UnitZ());
  yp = ComputeYawPitch(t.q()) / kDegreesToRadians;
  EXPECT_EIGEN_APPROX(yp, Vector2d(135, 0));

  t = util::LookAt({0, 0, 0}, {0, -1, 0}, Vector3d::UnitZ());
  yp = ComputeYawPitch(t.q()) / kDegreesToRadians;
  EXPECT_EIGEN_APPROX(yp, Vector2d(180, 0));

  t = util::LookAt({0, 0, 0}, {0, 0, 1}, Vector3d::UnitZ());
  yp = ComputeYawPitch(t.q()) / kDegreesToRadians;
  EXPECT_EIGEN_APPROX(yp, Vector2d(0, 90));

  t = util::LookAt({0, 0, 0}, {0, 1, 1}, Vector3d::UnitZ());
  yp = ComputeYawPitch(t.q()) / kDegreesToRadians;
  EXPECT_EIGEN_APPROX(yp, Vector2d(0, 45));

  t = util::LookAt({0, 0, 0}, {0, 1, -1}, Vector3d::UnitZ());
  yp = ComputeYawPitch(t.q()) / kDegreesToRadians;
  EXPECT_EIGEN_APPROX(yp, Vector2d(0, -45));

  t = util::LookAt({0, 0, 0}, {-1, -1, M_SQRT2}, Vector3d::UnitZ());
  yp = ComputeYawPitch(t.q()) / kDegreesToRadians;
  EXPECT_EIGEN_APPROX(yp, Vector2d(-135, 45));

  t = util::LookAt({0, 0, 0}, {0.001, 0, -1}, Vector3d::UnitZ());
  yp = ComputeYawPitch(t.q()) / kDegreesToRadians;
  EXPECT_EIGEN_APPROX(yp, Vector2d(90, -89.9427), 1e-3);
}

}  // namespace
}  // namespace guideline::util
