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

#include "project_guideline/depth/point_cloud_util.h"

#include <memory>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <opencv2/core.hpp>  // keep include
#include "project_guideline/camera/cv_camera_model.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/transformation.h"

namespace guideline::depth {
namespace {

using camera::CvCameraModel;
using ::Eigen::Vector3d;

CvCameraModel GetCameraModel() {
  return CvCameraModel(640, 480, Eigen::Vector4d(400., 400., 320.0, 240.0));
}

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

TEST(ArDepthToPointCloudTest, CreateFailsOnInvalidSubsampleRate) {
  // Sets invalid subsample rate.
  const int subsample_rate = -1;  // Invalid, must be positive.

  constexpr int kWidth = 3;
  constexpr int kHeight = 3;

  // Makes all extrinsic transforms the identity.
  util::Transformation look_forward = util::LookAt(
      /*camera_position=*/Vector3d::Zero(),
      /*look_at_point=*/Vector3d::UnitY(),
      /*up=*/Vector3d::UnitZ());

  util::DepthImageU16 depth_image = CreateDepthImage(kWidth, kHeight, 0);
  util::ConfidenceImageU8 confidence_image =
      CreateConfidenceImage(kWidth, kHeight, 0);
  ASSERT_DEATH(ArDepthToPointCloud(depth_image, confidence_image, look_forward,
                                   GetCameraModel(), subsample_rate)
                   .value(),
               "");
}

TEST(ArDepthToPointCloudTest, MismatchDepthImageAspectRatio) {
  // Sets invalid subsample rate.
  constexpr int kWidth = 32;
  constexpr int kHeight = 32;

  // Makes all extrinsic transforms the identity.
  util::Transformation look_forward = util::LookAt(
      /*camera_position=*/Vector3d::Zero(),
      /*look_at_point=*/Vector3d::UnitY(),
      /*up=*/Vector3d::UnitZ());

  util::DepthImageU16 depth_image = CreateDepthImage(kWidth, kHeight, 0);
  util::ConfidenceImageU8 confidence_image =
      CreateConfidenceImage(kWidth, kHeight, 0);
  ASSERT_DEATH(ArDepthToPointCloud(depth_image, confidence_image, look_forward,
                                   GetCameraModel())
                   .value(),
               "Image and depth map aspect ratios don't match");
}

TEST(ArDepthToPointCloudTest, ConvertSucceedsOnSimpleImage) {
  // Sets invalid subsample rate.
  constexpr int kWidth = 32;
  constexpr int kHeight = 24;

  // Makes all extrinsic transforms the identity.
  util::Transformation look_forward = util::LookAt(
      /*camera_position=*/Vector3d::Zero(),
      /*look_at_point=*/Vector3d::UnitY(),
      /*up=*/Vector3d::UnitZ());

  // Makes simple images to convert.
  constexpr uint16_t kDepthMm = 2000;  // Units: millimeters.
  util::DepthImageU16 depth_image = CreateDepthImage(kWidth, kHeight, kDepthMm);

  constexpr uint8_t kConfidenceUint8 = 100;
  util::ConfidenceImageU8 confidence_image =
      CreateConfidenceImage(kWidth, kHeight, kConfidenceUint8);

  GL_ASSERT_OK_AND_ASSIGN(std::vector<Point3D> point_cloud,
                          ArDepthToPointCloud(depth_image, confidence_image,
                                              look_forward, GetCameraModel()));
  ASSERT_EQ(point_cloud.size(), kWidth * kHeight);
  constexpr float kEpsilon = 0.001f;
  constexpr float kMmToM = 0.001f;
  for (int i = 0; i < point_cloud.size(); ++i) {
    // This point matches the following pixel coordinates.
    const int row = i / kWidth;
    const int col = i % kWidth;

    // Compares results.  Expects the output points to form a grid at uniform
    // depth.
    if (row > 0) {
      EXPECT_LT(point_cloud[i].coordinate(2),
                point_cloud[i - kWidth].coordinate(2));
    }
    if (col > 0) {
      EXPECT_GT(point_cloud[i].coordinate(0), point_cloud[i - 1].coordinate(0));
    }
    // Expect center pixel maps to (0, 2, 0) in world coordinate.
    if (row == kHeight / 2 && col == kWidth / 2) {
      EXPECT_NEAR(point_cloud[i].coordinate(0), 0.0f, kEpsilon);
      EXPECT_NEAR(point_cloud[i].coordinate(2), 0.0f, kEpsilon);
    }
    EXPECT_NEAR(point_cloud[i].coordinate(1), kDepthMm * kMmToM, kEpsilon);
    EXPECT_EQ(point_cloud[i].confidence, 100.0f / 255.0f);
  }
}

TEST(DepthMapToPointCloudTest, SimpleDepthMap) {
  // Makes all extrinsic transforms the identity.
  util::Transformation look_forward = util::LookAt(
      /*camera_position=*/Vector3d::Zero(),
      /*look_at_point=*/Vector3d::UnitY(),
      /*up=*/Vector3d::UnitZ());

  constexpr int kDepthMapWidth = 16;
  constexpr int kDepthMapHeight = 12;
  constexpr float kBackgroundDepthMeters = 100;

  auto depth_map_mat =
      std::make_unique<cv::Mat>(kDepthMapHeight, kDepthMapWidth, CV_32FC1);
  depth_map_mat->setTo(kBackgroundDepthMeters);

  constexpr int kObstacleSize = 4;
  constexpr int kObstacleDepthMeters = 2;

  for (int y = kDepthMapHeight / 2 - kObstacleSize / 2;
       y < kDepthMapHeight / 2 + kObstacleSize / 2; ++y) {
    for (int x = kDepthMapWidth / 2 - kObstacleSize / 2;
         x < kDepthMapWidth / 2 + kObstacleSize / 2; ++x) {
      depth_map_mat->at<float>(y, x) = kObstacleDepthMeters;
    }
  }

  util::DepthImage depth_map(kDepthMapWidth, kDepthMapHeight,
                             std::move(depth_map_mat));

  GL_ASSERT_OK_AND_ASSIGN(
      std::vector<Point3D> point_cloud,
      DepthMapToPointCloud(depth_map, look_forward, GetCameraModel()));
  ASSERT_EQ(point_cloud.size(), kDepthMapWidth * kDepthMapHeight);
  constexpr float kEpsilon = 0.001f;

  auto center_point =
      point_cloud[(kDepthMapHeight / 2) * kDepthMapWidth + kDepthMapWidth / 2];
  EXPECT_NEAR(center_point.coordinate(0), 0.0f, kEpsilon);
  EXPECT_NEAR(center_point.coordinate(1), kObstacleDepthMeters, kEpsilon);
  EXPECT_NEAR(center_point.coordinate(2), 0.0f, kEpsilon);

  auto corner_point = point_cloud[0];
  EXPECT_NEAR(corner_point.coordinate(0), -80, kEpsilon);
  EXPECT_NEAR(corner_point.coordinate(1), kBackgroundDepthMeters, kEpsilon);
  EXPECT_NEAR(corner_point.coordinate(2), 60.0f, kEpsilon);
}

}  // namespace
}  // namespace guideline::depth
