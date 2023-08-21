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

#include "project_guideline/depth/depth_align_ransac.h"

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <opencv2/core/mat.hpp>
#include "project_guideline/camera/cv_camera_model.h"
#include "project_guideline/motion/tracking_feature.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/image.h"

namespace guideline::depth {
namespace {

using camera::CvCameraModel;
using ::Eigen::Vector3d;
using motion::TrackingFeature;
using util::DepthImage;

constexpr int kDepthMapWidth = 16;
constexpr int kDepthMapHeight = 12;
constexpr int kImageWidth = 640;
constexpr int kImageHeight = 480;
constexpr float kBackgroundDepthMeters = 20.f;
constexpr float kScaleShiftEpsilon = 0.002f;
const Eigen::Vector2d kDepthMapScale{1.0 * kDepthMapWidth / kImageWidth,
                                     1.0 * kDepthMapHeight / kImageHeight};

const CvCameraModel* kCameraModel = new CvCameraModel(
    kImageWidth, kImageHeight,
    Eigen::Vector4d(400., 400., kImageWidth / 2.0, kImageHeight / 2.0));

float Randomize(float x, float max_dev, uint32_t* seed) {
  float f =
      (2.0f * static_cast<float>(rand_r(seed)) / static_cast<float>(RAND_MAX)) -
      1.0f;
  return x + (f * max_dev);
}

cv::Mat CreateRandomDepthMat(uint16_t width, uint16_t height, uint32_t* seed) {
  cv::Mat mat(height, width, CV_32FC1);
  for (auto iter = mat.begin<float>(); iter != mat.end<float>(); ++iter) {
    *iter = Randomize(20, 20, seed);
  }
  return mat;
}

float RawDepthToDepthMapValue(float depth, float scale, float shift) {
  return scale / ((1. / depth) - shift);
}

TEST(DepthAlignRansacTest, NoFeatures) {
  DepthImage depth_image(
      kDepthMapWidth, kDepthMapHeight,
      std::make_unique<cv::Mat>(kDepthMapHeight, kDepthMapWidth, CV_32FC1));
  std::vector<TrackingFeature> tracking_features;
  EXPECT_FALSE(
      DepthAlignRansac(depth_image, tracking_features, *kCameraModel).ok());
}

TEST(DepthAlignRansacTest, ConstantScaleShift) {
  uint32_t seed = 0;
  auto depth_mat = std::make_unique<cv::Mat>(
      CreateRandomDepthMat(kDepthMapWidth, kDepthMapHeight, &seed));

  constexpr float kDepthScale = 0.456;
  constexpr float kDepthShift = 0.0234;
  std::vector<TrackingFeature> tracking_features;
  for (int y = 0; y < kDepthMapHeight; y++) {
    for (int x = 0; x < kDepthMapWidth; x++) {
      float depth = depth_mat->at<float>(y, x);

      // Replace the metric depth value with the mapped inverse depth.
      depth_mat->at<float>(y, x) =
          RawDepthToDepthMapValue(depth, kDepthScale, kDepthShift);

      Vector3d ray;
      kCameraModel->PixelToRay({x / kDepthMapScale.x(), y / kDepthMapScale.y()},
                               ray);
      Vector3d camera_t_feature = (depth / ray.z()) * ray;
      tracking_features.emplace_back(camera_t_feature.cast<float>(), 1.0f, 0);
    }
  }

  DepthImage depth_image(kDepthMapWidth, kDepthMapHeight, std::move(depth_mat));

  GL_ASSERT_OK_AND_ASSIGN(
      auto params,
      DepthAlignRansac(depth_image, tracking_features, *kCameraModel));

  ASSERT_NEAR(params.scale, kDepthScale, kScaleShiftEpsilon);
  ASSERT_NEAR(params.shift, kDepthShift, kScaleShiftEpsilon);
}

TEST(DepthAlignRansacTest, SameDepth) {
  uint32_t seed = 0;
  auto depth_mat = std::make_unique<cv::Mat>(
      CreateRandomDepthMat(kDepthMapWidth, kDepthMapHeight, &seed));
  DepthImage depth_image(kDepthMapWidth, kDepthMapHeight, std::move(depth_mat));

  std::vector<TrackingFeature> tracking_features;

  for (int y = 0; y < kDepthMapHeight; y++) {
    for (int x = 0; x < kDepthMapWidth; x++) {
      Vector3d ray;
      kCameraModel->PixelToRay({x / kDepthMapScale.x(), y / kDepthMapScale.y()},
                               ray);
      Vector3d camera_t_feature = (depth_image(y, x) / ray.z()) * ray;
      tracking_features.emplace_back(camera_t_feature.cast<float>(), 1.0f, 0);
    }
  }

  GL_ASSERT_OK_AND_ASSIGN(
      auto params,
      DepthAlignRansac(depth_image, tracking_features, *kCameraModel));

  ASSERT_NEAR(params.scale, 1.0f, kScaleShiftEpsilon);
  ASSERT_NEAR(params.shift, 0.0f, kScaleShiftEpsilon);
}

TEST(DepthAlignRansacTest, ConstantScaleShiftWithOutliers) {
  uint32_t seed = 0;

  auto depth_mat = std::make_unique<cv::Mat>(
      CreateRandomDepthMat(kDepthMapWidth, kDepthMapHeight, &seed));
  constexpr float kDepthScale = 0.456;
  constexpr float kDepthShift = 0.0234;
  std::vector<TrackingFeature> tracking_features;
  for (int y = 0; y < kDepthMapHeight; y++) {
    for (int x = 0; x < kDepthMapWidth; x++) {
      int idx = y * kDepthMapWidth + x;
      float depth = depth_mat->at<float>(y, x);

      float map_depth = depth;
      // Insert some bogus depths.
      if (idx % 4 == 0) {
        map_depth = Randomize(20, 20, &seed);
      }
      depth_mat->at<float>(y, x) =
          RawDepthToDepthMapValue(map_depth, kDepthScale, kDepthShift);

      constexpr int kMaxTrackingFeatures = 100;
      if (tracking_features.size() < kMaxTrackingFeatures) {
        Vector3d ray;
        kCameraModel->PixelToRay(
            {x / kDepthMapScale.x(), y / kDepthMapScale.y()}, ray);
        Vector3d camera_t_feature = (depth / ray.z()) * ray;
        tracking_features.emplace_back(camera_t_feature.cast<float>(), 1.0f, 0);
      }
    }
  }

  DepthImage depth_image(kDepthMapWidth, kDepthMapHeight, std::move(depth_mat));

  GL_ASSERT_OK_AND_ASSIGN(
      auto params,
      DepthAlignRansac(depth_image, tracking_features, *kCameraModel));

  ASSERT_NEAR(params.scale, kDepthScale, kScaleShiftEpsilon);
  ASSERT_NEAR(params.shift, kDepthShift, kScaleShiftEpsilon);
}

TEST(DepthAlignRansacTest, RandomScaleShiftVariation) {
  uint32_t seed = 0;

  auto depth_mat = std::make_unique<cv::Mat>(
      CreateRandomDepthMat(kDepthMapWidth, kDepthMapHeight, &seed));

  constexpr float kDepthScale = 0.456;
  constexpr float kDepthShift = 0.0234;
  constexpr float kDepthScaleTolerance = 0.1;
  constexpr float kDepthShiftTolerance = 0.01;

  std::vector<TrackingFeature> tracking_features;
  for (int y = 0; y < kDepthMapHeight; y++) {
    for (int x = 0; x < kDepthMapWidth; x++) {
      float depth = depth_mat->at<float>(y, x);

      float scale = Randomize(kDepthScale, kDepthScaleTolerance / 2, &seed);
      float shift = Randomize(kDepthShift, kDepthShiftTolerance / 2, &seed);
      depth_mat->at<float>(y, x) = RawDepthToDepthMapValue(depth, scale, shift);

      Vector3d ray;
      kCameraModel->PixelToRay({x / kDepthMapScale.x(), y / kDepthMapScale.y()},
                               ray);
      Vector3d camera_t_feature = (depth / ray.z()) * ray;
      tracking_features.emplace_back(camera_t_feature.cast<float>(), 1.0f, 0);
    }
  }

  DepthImage depth_image(kDepthMapWidth, kDepthMapHeight, std::move(depth_mat));

  GL_ASSERT_OK_AND_ASSIGN(
      auto params,
      DepthAlignRansac(depth_image, tracking_features, *kCameraModel));

  ASSERT_NEAR(params.scale, kDepthScale, kDepthScaleTolerance);
  ASSERT_NEAR(params.shift, kDepthShift, kDepthShiftTolerance);
}

TEST(DepthAlignRansacTest, NotEnoughInliers) {
  uint32_t seed = 0;

  auto depth_mat = std::make_unique<cv::Mat>(
      CreateRandomDepthMat(kDepthMapWidth, kDepthMapHeight, &seed));

  constexpr float kDepthScale = 0.456;
  constexpr float kDepthShift = 0.002;
  std::vector<TrackingFeature> tracking_features;
  for (int y = 0; y < kDepthMapHeight; y++) {
    for (int x = 0; x < kDepthMapWidth; x++) {
      float depth = depth_mat->at<float>(y, x);

      // Assign random depth map values.
      depth_mat->at<float>(y, x) = RawDepthToDepthMapValue(
          Randomize(4, 4, &seed), kDepthScale, kDepthShift);

      constexpr int kMaxTrackingFeatures = 12;
      if (tracking_features.size() < kMaxTrackingFeatures) {
        Vector3d ray;
        kCameraModel->PixelToRay(
            {x / kDepthMapScale.x(), y / kDepthMapScale.y()}, ray);
        Vector3d camera_t_feature = (depth / ray.z()) * ray;
        tracking_features.emplace_back(camera_t_feature.cast<float>(), 1.0f, 0);
      }
    }
  }

  DepthImage depth_image(kDepthMapWidth, kDepthMapHeight, std::move(depth_mat));

  EXPECT_FALSE(
      DepthAlignRansac(depth_image, tracking_features, *kCameraModel).ok());
}

TEST(DepthAlignRansacTest, NotEnoughFeatures) {
  uint32_t seed = 0;
  auto depth_mat = std::make_unique<cv::Mat>(
      CreateRandomDepthMat(kDepthMapWidth, kDepthMapHeight, &seed));

  std::vector<TrackingFeature> tracking_features;
  tracking_features.emplace_back(Eigen::Vector3f{1.0f, 2.0f, 3.0f}, 1.0f, 0);

  DepthImage depth_image(kDepthMapWidth, kDepthMapHeight, std::move(depth_mat));

  EXPECT_FALSE(
      DepthAlignRansac(depth_image, tracking_features, *kCameraModel).ok());
}

}  // namespace
}  // namespace guideline::depth
