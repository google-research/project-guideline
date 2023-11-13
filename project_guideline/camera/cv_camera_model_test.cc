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

#include "project_guideline/camera/cv_camera_model.h"

#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "Eigen/Core"  // keep include
#include "project_guideline/testing/predicates.h"

namespace guideline::camera {
namespace {
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

TEST(CvCameraModel, PointToPixelPinhole) {
  CvCameraModel model(640, 480, {100, 100, 320, 240});
  Vector2d pixel;
  EXPECT_TRUE(model.PointToPixel({0, 0, 1}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(320, 240));

  EXPECT_TRUE(model.PointToPixel({1, 1, 1}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(420, 340));

  EXPECT_TRUE(model.PointToPixel({1, 1, 5}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(340, 260));

  EXPECT_TRUE(model.PointToPixel({2, 2, 5}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(360, 280));

  EXPECT_TRUE(model.PointToPixel({5, 5, 1}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(820, 740));

  EXPECT_TRUE(model.PointToPixel({5, 5, 1e10}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(320, 240), 1e-6);

  EXPECT_TRUE(model.PointToPixel({-1, -1, 1}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(220, 140));

  EXPECT_FALSE(model.PointToPixel(
      {5, 5, std::numeric_limits<double>::infinity()}, pixel));
  EXPECT_FALSE(model.PointToPixel(
      {std::numeric_limits<double>::infinity(), 1, 1}, pixel));

  // Point behind camera fails.
  EXPECT_FALSE(model.PointToPixel({1, 1, -1}, pixel));
  EXPECT_FALSE(model.PointToPixel({1, 1, 0}, pixel));
}

TEST(CvCameraModel, PointToPixelDistortion) {
  CvCameraModel model(640, 480, {100, 100, 320, 240},
                      Vector4d(0.01, 0.02, 0.03, 0.04));
  Vector2d pixel;
  EXPECT_TRUE(model.PointToPixel({0, 0, 1}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(320, 240));

  EXPECT_TRUE(model.PointToPixel({1, 1, 1}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(452, 370));

  EXPECT_TRUE(model.PointToPixel({1, 1, 5}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(340.89856, 260.81856));

  EXPECT_TRUE(model.PointToPixel({2, 2, 5}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(363.72992, 283.40992));

  EXPECT_TRUE(model.PointToPixel({5, 5, 1}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(26620, 26490));

  EXPECT_TRUE(model.PointToPixel({5, 5, 1e10}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(320, 240), 1e-6);

  EXPECT_TRUE(model.PointToPixel({-1, -1, 1}, pixel));
  EXPECT_EIGEN_APPROX(pixel, Vector2d(232, 150));

  EXPECT_FALSE(model.PointToPixel(
      {5, 5, std::numeric_limits<double>::infinity()}, pixel));
  EXPECT_FALSE(model.PointToPixel(
      {std::numeric_limits<double>::infinity(), 1, 1}, pixel));

  // Point behind camera fails.
  EXPECT_FALSE(model.PointToPixel({1, 1, -1}, pixel));
  EXPECT_FALSE(model.PointToPixel({1, 1, 0}, pixel));
}

TEST(CvCameraModel, PixelToRayPinhole) {
  CvCameraModel model(640, 480, {100, 100, 320, 240}, Vector4d::Zero());
  Vector3d ray;
  EXPECT_TRUE(model.PixelToRay({320, 240}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(0, 0, 1));

  EXPECT_TRUE(model.PixelToRay({0, 0}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(-3.2, -2.4, 1));

  EXPECT_TRUE(model.PixelToRay({320, 240}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(0, 0, 1));

  EXPECT_TRUE(model.PixelToRay({640, 480}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(3.2, 2.4, 1));

  EXPECT_TRUE(model.PixelToRay({360, 180}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(0.4, -0.6, 1));

  EXPECT_TRUE(model.PixelToRay({-1000, -1000}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(-13.2, -12.4, 1));

  EXPECT_TRUE(model.PixelToRay({1000, 1000}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(6.8, 7.6, 1));

  EXPECT_FALSE(
      model.PixelToRay({1, std::numeric_limits<double>::infinity()}, ray));
}

TEST(CvCameraModel, PixelToRayDistortion) {
  CvCameraModel model(640, 480, {100, 100, 320, 240},
                      Vector4d(0.01, 0.02, 0.03, 0.04));
  Vector3d ray;
  EXPECT_TRUE(model.PixelToRay({320, 240}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(0, 0, 1));

  EXPECT_TRUE(model.PixelToRay({0, 0}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(-0.80033, -0.600246, 1), 1e-4);

  EXPECT_TRUE(model.PixelToRay({320, 240}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(0, 0, 1));

  EXPECT_TRUE(model.PixelToRay({640, 480}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(0.208096, 0.156072, 1), 1e-4);

  EXPECT_TRUE(model.PixelToRay({360, 180}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(0.377746, -0.6131065, 1), 1e-4);

  EXPECT_TRUE(model.PixelToRay({-1000, -1000}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(-0.0232287, -0.020668, 1), 1e-4);

  EXPECT_TRUE(model.PixelToRay({1000, 1000}, ray));
  EXPECT_EIGEN_APPROX(ray, Vector3d(-0.019051, -0.014288, 1), 1e-4);

  EXPECT_FALSE(
      model.PixelToRay({1, std::numeric_limits<double>::infinity()}, ray));
}

TEST(CvCameraModel, PointToPixelAndBackPinhole) {
  std::vector<Vector3d> camera_points = {
      {0, 0, 1},  {1, 1, 1},  {-1, -1, 1}, {1, 1, 5},   {5, 5, 100},
      {1, 2, 3},  {-1, 2, 3}, {1, -2, 3},  {-1, -2, 3}, {2, 2, 1e-3},
      {-5, 2, 2}, {2, -5, 2}, {3, 3, 1e10}};
  CvCameraModel model(640, 480, {100, 100, 320, 240});

  Vector2d pixel;
  Vector3d ray;
  for (const auto& camera_point : camera_points) {
    EXPECT_TRUE(model.PointToPixel(camera_point, pixel)) << camera_point;
    EXPECT_TRUE(model.PixelToRay(pixel, ray)) << camera_point;
    EXPECT_EIGEN_APPROX((camera_point.z() / ray.z()) * ray, camera_point, 1e-4);
  }
}

TEST(CvCameraModel, PointToPixelAndBackDistortion) {
  std::vector<Vector3d> camera_points = {
      {0, 0, 1}, {1, 1, 2},  {-1, -1, 1}, {1, 1, 5},   {5, 5, 100},
      {1, 2, 3}, {-1, 2, 3}, {1, -2, 3},  {-1, -2, 3}, {3, 3, 1e10}};
  CvCameraModel model(640, 480, {100, 100, 320, 240},
                      Vector4d(0.01, 0.02, 0.03, 0.04));

  Vector2d pixel;
  Vector3d ray;
  for (const auto& camera_point : camera_points) {
    EXPECT_TRUE(model.PointToPixel(camera_point, pixel)) << camera_point;
    EXPECT_TRUE(model.PixelToRay(pixel, ray)) << camera_point;
    EXPECT_EIGEN_APPROX((camera_point.z() / ray.z()) * ray, camera_point, 1e-3);
  }
}

}  // namespace
}  // namespace guideline::camera
