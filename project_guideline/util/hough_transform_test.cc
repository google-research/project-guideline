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

#include "project_guideline/util/hough_transform.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "Eigen/Core"  // keep include
#include "project_guideline/testing/predicates.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/image_io.h"
#include "project_guideline/util/status.h"

namespace guideline::util {
namespace {

const char kTestDataDir[] =
    "/project_guideline/project_guideline/vision/testdata/";

constexpr int kMaskWidth = 65;
constexpr int kMaskHeight = 65;
constexpr int kAngleSteps = 180;
constexpr float kConfidenceThreshold = 0.2;
constexpr float kSegmentOverlapPct = 0.1;
constexpr int kImageWidth = 800;
constexpr int kImageHeight = 600;

using ::Eigen::Vector2f;

std::string GetTestDataPath(const std::string& filename) {
  return std::string(getenv("TEST_SRCDIR")) + kTestDataDir + filename;
}

absl::StatusOr<cv::Mat> LoadMaskGolden() {
  constexpr double kMaskImageScale = 65535;
  GL_ASSIGN_OR_RETURN(
      auto mask_golden,
      LoadFloatImagePng(GetTestDataPath("guideline1_mask_golden.png"),
                        kMaskImageScale));
  cv::Mat mask(kMaskHeight, kMaskWidth, CV_32F);
  cv::resize(mask_golden.data(), mask, mask.size(), 0, 0, cv::INTER_LINEAR);
  return mask;
}

TEST(HoughTransformTest, StraightLine) {
  const int kNumSegments = 1;
  HoughTransform hough_transform(kMaskWidth, kAngleSteps, kConfidenceThreshold,
                                 kMaskWidth, kMaskHeight, kNumSegments,
                                 kSegmentOverlapPct);

  const int xStart = 15;
  const int xEnd = 16;
  cv::Mat mask(kMaskHeight, kMaskWidth, CV_32F);
  mask.setTo(0);
  for (int y = 0; y < kMaskHeight; ++y) {
    for (int x = xStart; x <= xEnd; ++x) {
      mask.at<float>(y, x) = 1.0f;
    }
  }

  std::vector<HoughTransformResult> results = hough_transform.Process(mask);
  ASSERT_EQ(results.size(), 1);

  const float kMidX = (xStart + xEnd) / 2.0f;
  EXPECT_EIGEN_APPROX(results[0].bottom_intercept,
                      Vector2f(kMidX / kMaskWidth, 1), 1e-4);
  EXPECT_EIGEN_APPROX(results[0].top_intercept, Vector2f(kMidX / kMaskWidth, 0),
                      1e-4);
  EXPECT_NEAR(results[0].GetAngleDegrees(kImageWidth, kImageHeight), 0.0, 1e-4);
  EXPECT_GT(results[0].score, 30);
}

TEST(HoughTransformTest, MaskGoldenSingleSegment) {
  const int kNumSegments = 1;
  HoughTransform hough_transform(kMaskWidth, kAngleSteps, kConfidenceThreshold,
                                 kMaskWidth, kMaskHeight, kNumSegments,
                                 kSegmentOverlapPct);

  GL_ASSERT_OK_AND_ASSIGN(cv::Mat mask, LoadMaskGolden());
  std::vector<HoughTransformResult> results = hough_transform.Process(mask);
  ASSERT_EQ(results.size(), 1);

  EXPECT_EIGEN_APPROX(results[0].bottom_intercept, Vector2f(0.29154, 1), 1e-4);
  EXPECT_EIGEN_APPROX(results[0].top_intercept, Vector2f(0.45888, 0), 1e-4);
  EXPECT_NEAR(results[0].GetAngleDegrees(kImageWidth, kImageHeight), 12.578,
              1e-4);
  EXPECT_NEAR(results[0].score, 40.22, 1e-2);
}

TEST(HoughTransformTest, MaskGoldenCurve) {
  const int kNumSegments = 3;
  HoughTransform hough_transform(kMaskWidth, kAngleSteps, kConfidenceThreshold,
                                 kMaskWidth, kMaskHeight, kNumSegments,
                                 kSegmentOverlapPct);

  GL_ASSERT_OK_AND_ASSIGN(cv::Mat mask, LoadMaskGolden());
  std::vector<HoughTransformResult> results = hough_transform.Process(mask);
  ASSERT_EQ(results.size(), 3);

  EXPECT_EIGEN_APPROX(results[0].bottom_intercept, Vector2f(0.378107, 0.4),
                      1e-4);
  EXPECT_EIGEN_APPROX(results[0].top_intercept, Vector2f(0.890083, 0), 1e-4);
  EXPECT_NEAR(results[0].GetAngleDegrees(kImageWidth, kImageHeight), 59.6312,
              1e-4);
  EXPECT_NEAR(results[0].score, 41.08, 1e-2);

  EXPECT_EIGEN_APPROX(results[1].bottom_intercept, Vector2f(0.326447, 0.692308),
                      1e-4);
  EXPECT_EIGEN_APPROX(results[1].top_intercept, Vector2f(0.452567, 0.292308),
                      1e-4);
  EXPECT_NEAR(results[1].GetAngleDegrees(kImageWidth, kImageHeight), 22.8018,
              1e-4);
  EXPECT_NEAR(results[1].score, 55.45, 1e-2);

  EXPECT_EIGEN_APPROX(results[2].bottom_intercept, Vector2f(0.293992, 0.984615),
                      1e-4);
  EXPECT_EIGEN_APPROX(results[2].top_intercept, Vector2f(0.348681, 0.584615),
                      1e-4);
  EXPECT_NEAR(results[2].GetAngleDegrees(kImageWidth, kImageHeight), 10.3315,
              1e-4);
  EXPECT_NEAR(results[2].score, 65.49, 1e-2);
}

}  // namespace
}  // namespace guideline::util
