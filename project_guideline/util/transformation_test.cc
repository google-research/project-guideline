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

#include "project_guideline/util/transformation.h"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "Eigen/Core"      // keep include
#include "Eigen/Geometry"  // keep include
#include "project_guideline/testing/predicates.h"

namespace guideline::util {
namespace {

static constexpr double kDoubleTolerance = 1e-7;
constexpr double kDegreesToRadians = M_PI / 180.0;

static const std::vector<Quaternion> kTestQuaternions = {
    {1, 0, 0, 0},
    {0, -1, 0, 0},
    {0, 0, -1, 0},
    {0, 0, 0, -1},
    {1, -1, -1, -1},
    {1, 1, 1, 1},
    {0, -1, 2, 3},
    {0, 0, -1, 2},
    {1.409, -0.537, 0.433, -0.725},
    {1.417, -1.833, -0.342, 0.063},
    {0.671, 2.258, -3.578, -0.714},
    {1.207, -0.862, -2.769, 0.205},
    {0.717, -0.318, 1.349, 0.124},
    {1.630, 1.307, -3.034, -1.489}};

static const std::vector<Vector3> kTestPoints = {
    {0, 0, 0},  {1, 0, 0},  {0, 1, 0},       {0, 0, 1},
    {1, -1, 0}, {1, -1, 1}, {1, 1e-2, 1e-4}, {0.53, -2.25, 0.31}};

std::vector<Transformation> GetTestTransformations() {
  std::vector<Transformation> test_transforms;
  test_transforms.reserve(kTestQuaternions.size() * kTestPoints.size());
  for (auto q : kTestQuaternions) {
    for (const auto& p : kTestPoints) {
      test_transforms.emplace_back(q, p);
    }
  }
  return test_transforms;
}

TEST(Transformation, DefaultConstructor) {
  Transformation t;
  ASSERT_EQ(t, Transformation::Identity());
  ASSERT_EIGEN_APPROX(t.p(), Vector3::Zero(), kDoubleTolerance);

  ASSERT_NEAR(t.q().x(), 0, kDoubleTolerance);
  ASSERT_NEAR(t.q().y(), 0, kDoubleTolerance);
  ASSERT_NEAR(t.q().z(), 0, kDoubleTolerance);
  ASSERT_NEAR(t.q().w(), 1, kDoubleTolerance);
}

TEST(Transformation, TranslationAddition) {
  Transformation t({1, 1, 1, 1}, {1, 2, 3});
  Transformation t1 = t + Vector3(4, 5, 6);
  ASSERT_EIGEN_APPROX(t1.p(), Vector3(5.0, 7.0, 9.0), kDoubleTolerance);
}

TEST(Transformation, Compose) {
  Transformation t1({1.417, -1.833, -0.342, 0.063}, {120.523, -56.34, 0.0003});
  Transformation t2({0.671, 2.258, -3.578, -0.714}, {-5.67, 82.334, 19.456});
  Transformation composed = t1 * t2;
  ASSERT_EIGEN_APPROX(composed.p(),
                      Vector3(126.977622379, -58.144311571, -84.525759305),
                      kDoubleTolerance);
  ASSERT_EIGEN_APPROX(
      composed.q().coeffs(),
      Eigen::Vector4d(0.239740557, -0.635510692, 0.625213302, 0.384394266),
      kDoubleTolerance);
}

TEST(Transformation, ComposeWithRotations) {
  Transformation t1 = Transformation(
      Quaternion(Eigen::AngleAxisd(90 * kDegreesToRadians, Vector3::UnitX())),
      {1, 1, 1});

  Transformation t2 = Transformation(
      Quaternion(Eigen::AngleAxisd(270 * kDegreesToRadians, Vector3::UnitZ())),
      {-1, -1, -1});

  Transformation t12 = t1 * t2;
  ASSERT_EIGEN_APPROX(t12.p(), Vector3(0.0, 2.0, 0.0), kDoubleTolerance);
  ASSERT_EIGEN_APPROX(t12.q().coeffs(), Eigen::Vector4d(-0.5, -0.5, 0.5, -0.5),
                      kDoubleTolerance);

  Transformation t21 = t2 * t1;
  ASSERT_EIGEN_APPROX(t21.p(), Vector3(0.0, -2.0, 0.0), kDoubleTolerance);
  ASSERT_EIGEN_APPROX(t21.q().coeffs(), Eigen::Vector4d(-0.5, 0.5, 0.5, -0.5),
                      kDoubleTolerance);
}

TEST(Transformation, QuaternionCompose) {
  Vector3 unit_x = Vector3::UnitX();
  Transformation t =
      Transformation(unit_x) *
      Quaternion(Eigen::AngleAxisd(45 * kDegreesToRadians, Vector3::UnitX()));
  ASSERT_EIGEN_APPROX(t.p(), Vector3(1.0, 0.0, 0.0), kDoubleTolerance);
  ASSERT_EIGEN_APPROX(t * unit_x, Vector3(2.0, 0.0, 0.0), kDoubleTolerance);

  t = t *
      Quaternion(Eigen::AngleAxisd(90 * kDegreesToRadians, Vector3::UnitY()));
  ASSERT_EIGEN_APPROX(t.p(), Vector3(1.0, 0.0, 0.0), kDoubleTolerance);
  ASSERT_EIGEN_APPROX(t * unit_x, Vector3(1.0, 0.70710678, -0.70710678),
                      kDoubleTolerance);

  t = t *
      Quaternion(Eigen::AngleAxisd(-90 * kDegreesToRadians, Vector3::UnitZ()));
  ASSERT_EIGEN_APPROX(t.p(), Vector3(1.0, 0.0, 0.0), kDoubleTolerance);
  ASSERT_EIGEN_APPROX(t * unit_x, Vector3(1.0, -0.70710678, -0.70710678),
                      kDoubleTolerance);
}

TEST(Transformation, ToMatrix4x4) {
  Eigen::Vector4d unit_x = Eigen::Vector4d::UnitX();
  Transformation t = Transformation(
      Quaternion(Eigen::AngleAxisd(45 * kDegreesToRadians, Vector3::UnitX())));
  ASSERT_EIGEN_APPROX(t.ToMatrix4x4() * unit_x,
                      Eigen::Vector4d(1.0, 0.0, 0.0, 0.0), kDoubleTolerance);

  t = t *
      Quaternion(Eigen::AngleAxisd(90 * kDegreesToRadians, Vector3::UnitY()));
  ASSERT_EIGEN_APPROX(t.ToMatrix4x4() * unit_x,
                      Eigen::Vector4d(0.0, 0.70710678, -0.70710678, 0.0),
                      kDoubleTolerance);

  t = t *
      Quaternion(Eigen::AngleAxisd(-90 * kDegreesToRadians, Vector3::UnitZ()));
  ASSERT_EIGEN_APPROX(t.ToMatrix4x4() * unit_x,
                      Eigen::Vector4d(0.0, -0.70710678, -0.70710678, 0.0),
                      kDoubleTolerance);
}

TEST(Transformation, InverseOfInverse) {
  const auto transformations = GetTestTransformations();
  for (const auto& t : transformations) {
    const Transformation t_inverse = t.Inverse();
    const Transformation t_inverse_inverse = t_inverse.Inverse();
    ASSERT_EIGEN_APPROX(t.ToMatrix4x4(), t_inverse_inverse.ToMatrix4x4(),
                        kDoubleTolerance);
  }
}

TEST(Transformation, InverseComposedWithInverse) {
  const auto transformations = GetTestTransformations();
  for (const auto& t : transformations) {
    ASSERT_EIGEN_APPROX((t * t.Inverse()).ToMatrix4x4(),
                        Transformation::Identity().ToMatrix4x4(),
                        kDoubleTolerance);
  }
}


}  // namespace
}  // namespace guideline::util
