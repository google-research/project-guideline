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

#include "project_guideline/environment/point_cloud.h"

#include <memory>
#include <utility>

#include "gtest/gtest.h"
#include "Eigen/Core"
#include "project_guideline/camera/cv_camera_model.h"
#include "project_guideline/depth/point_cloud_util.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/transformation.h"

namespace guideline::environment {

namespace {
using depth::Point3D;
using ::Eigen::Vector3d;
using util::Transformation;

static const camera::CvCameraModel* kCameraModel =
    new camera::CvCameraModel(640, 480, {400., 400., 320.0, 240.0});

util::DepthImageU16 CreateDepthImage(int width, int height, uint16_t values) {
  cv::Mat depth_mat(height, width, CV_16UC1);
  depth_mat = values;
  return util::DepthImageU16(width, height,
                             std::make_unique<cv::Mat>(std::move(depth_mat)));
}

util::ConfidenceImageU8 CreateConfidenceImage(int width, int height,
                                              uint8_t values) {
  cv::Mat confidence_mat(height, width, CV_8UC1);
  confidence_mat = values;
  return util::ConfidenceImageU8(
      width, height, std::make_unique<cv::Mat>(std::move(confidence_mat)));
}

TEST(FrameBasedPointCloud, TestNormalBehaviour) {
  // Sets invalid subsample rate.
  constexpr int kWidth = 32;
  constexpr int kHeight = 24;

  // Makes all extrinsic transforms the identity.
  Transformation look_forward = util::LookAt(
      /*src=*/Vector3d::Zero(),
      /*look_at=*/Vector3d::UnitY(),
      /*up=*/Vector3d::UnitZ());

  // Makes simple images to convert.
  constexpr uint16_t kDepthMm = 2000;  // Units: millimeters.
  auto depth_image = CreateDepthImage(kWidth, kHeight, kDepthMm);

  constexpr uint8_t kConfidenceUint8 = 100;
  auto confidence_image =
      CreateConfidenceImage(kWidth, kHeight, kConfidenceUint8);

  PointCloudOptions options;
  auto point_cloud = PointCloud::Create(options);
  point_cloud->UpdatePointCloud(depth_image, confidence_image, look_forward,
                                *kCameraModel);
  GL_ASSERT_OK_AND_ASSIGN(std::vector<Point3D> converted_point_cloud,
                          point_cloud->GetPointCloud());
  ASSERT_EQ(converted_point_cloud.size(), kWidth * kHeight);

  constexpr float kEpsilon = 0.001f;
  constexpr float kMmToM = 0.001f;
  for (int i = 0; i < converted_point_cloud.size(); ++i) {
    // This point matches the following pixel coordinates.
    const int row = i / kWidth;
    const int col = i % kWidth;

    // Compares results.  Expects the output points to form a grid at uniform
    // depth.
    if (row > 0) {
      EXPECT_LT(converted_point_cloud[i].coordinate(2),
                converted_point_cloud[i - kWidth].coordinate(2));
    }
    if (col > 0) {
      EXPECT_GT(converted_point_cloud[i].coordinate(0),
                converted_point_cloud[i - 1].coordinate(0));
    }
    // Expect center pixel maps to (0, 2, 0) in world coordinate.
    if (row == kHeight / 2 && col == kWidth / 2) {
      EXPECT_NEAR(converted_point_cloud[i].coordinate(0), 0.0f, kEpsilon);
      EXPECT_NEAR(converted_point_cloud[i].coordinate(2), 0.0f, kEpsilon);
    }
    EXPECT_NEAR(converted_point_cloud[i].coordinate(1), kDepthMm * kMmToM,
                kEpsilon);
    EXPECT_EQ(converted_point_cloud[i].confidence, 100.0f / 255.0f);
  }
}

TEST(FrameBasedPointCloud, TestError) {
  PointCloudOptions options;
  auto point_cloud = PointCloud::Create(options);
  EXPECT_FALSE(point_cloud->GetPointCloud().ok());
}

}  // namespace
}  // namespace guideline::environment
