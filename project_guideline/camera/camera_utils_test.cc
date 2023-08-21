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

#include "project_guideline/camera/camera_utils.h"

#include "gtest/gtest.h"
#include "Eigen/Core"
#include "project_guideline/camera/camera_utils.h"
#include "project_guideline/camera/cv_camera_model.h"
#include "project_guideline/testing/predicates.h"

namespace guideline::camera {
namespace {

TEST(CameraUtils, ProjectionMatrix) {
  CvCameraModel model1(200, 400, {1, 1, 100, 200});
  Eigen::Matrix4d projection;
  GlCameraProjectionMatrix(model1, /*near=*/1, /*far=*/2, projection);

  Eigen::Matrix4d expected;
  expected << 0.01, 0, 0, 0, 0, 0.005, 0, 0, 0, 0, -3, -4, 0, 0, -1, 0;
  EXPECT_EIGEN_APPROX(projection, expected, 1e-5);

  CvCameraModel model2(640, 480, {400, 400, 322.567, 248.123});
  GlCameraProjectionMatrix(model2, /*near=*/1, /*far=*/100, projection);

  expected << 1.25, 0, -0.00802187, 0, 0, 1.66667, 0.0338458, 0, 0, 0, -1.0202,
      -2.0202, 0, 0, -1, 0;
  EXPECT_EIGEN_APPROX(projection, expected, 1e-5);

  CvCameraModel model3(640, 480, {200, 200, 301.567, 401.123});
  GlCameraProjectionMatrix(model3, /*near=*/1, /*far=*/100, projection);

  expected << 0.625, 0, 0.0576031, 0, 0, 0.833333, 0.671346, 0, 0, 0, -1.0202,
      -2.0202, 0, 0, -1, 0;
  EXPECT_EIGEN_APPROX(projection, expected, 1e-5);
}

}  // namespace
}  // namespace guideline::camera
