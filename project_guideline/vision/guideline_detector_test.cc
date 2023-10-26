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

#include "project_guideline/vision/guideline_detector.h"

#include <memory>
#include <optional>
#include <vector>

#include "gtest/gtest.h"
#include "absl/functional/bind_front.h"
#include "absl/synchronization/notification.h"
#include "Eigen/Core"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/testing/images.h"
#include "project_guideline/testing/predicates.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/image_io.h"

namespace guideline::vision {
namespace {

using util::Image;

const char kTestDataDir[] =
    "/project_guideline/project_guideline/vision/testdata/";

std::string GetTestDataPath(const std::string& filename) {
  return std::string(getenv("TEST_SRCDIR")) + kTestDataDir + filename;
}

struct TestDetectionCallback {
  void OnDetection(const int64_t timestamp_us,
                   const std::vector<Eigen::Vector3f>& keypoints,
                   std::shared_ptr<const util::ConfidenceMask> guideline_mask,
                   std::shared_ptr<const util::DepthImage> depth_map) {
    callback_timestamp_us = timestamp_us;
    callback_keypoints = keypoints;
    callback_guideline_mask = guideline_mask;
    callback_depth_map = depth_map;
    notification.Notify();
  }

  absl::Notification notification;
  int64_t callback_timestamp_us;
  std::optional<std::vector<Eigen::Vector3f>> callback_keypoints;
  std::shared_ptr<const util::ConfidenceMask> callback_guideline_mask = nullptr;
  std::shared_ptr<const util::DepthImage> callback_depth_map = nullptr;
};

TEST(GuidelineDetector, DepthMap) {
  GL_ASSERT_OK_AND_ASSIGN(
      auto image,
      util::LoadImage(GetTestDataPath("obstacle_synthetic.png"),
                      /*keep_alpha=*/false, util::kReverseLandscapeRotation));

  constexpr uint64_t kImageTimestampNs = 111222333444555;
  image.mutable_metadata().timestamp_ns = kImageTimestampNs;
  auto shared_image = std::make_shared<util::Image>(std::move(image));

  GuidelineEngineConfig_DetectorOptions options;
  GL_ASSERT_OK_AND_ASSIGN(auto detector, GuidelineDetector::Create(options));

  TestDetectionCallback callback;
  detector->AddCallback(
      absl::bind_front(&TestDetectionCallback::OnDetection, &callback));

  GL_ASSERT_OK(detector->Start());

  detector->OnImage(shared_image);

  callback.notification.WaitForNotificationWithTimeout(absl::Seconds(10));

  EXPECT_EQ(callback.callback_timestamp_us, kImageTimestampNs / 1000);
  ASSERT_TRUE(callback.callback_depth_map != nullptr);

  constexpr double kDepthImageScale = 1000.;
  constexpr double kDepthImageShift = 0.;

  // Uncomment to update the golden image.
  // GL_ASSERT_OK(
  //     util::SaveFloatImagePng(*callback.callback_depth_map,
  //     "/tmp/obstacle_synthetic_depth_golden.png", kDepthImageScale));

  GL_ASSERT_OK_AND_ASSIGN(
      auto depth_golden,
      util::LoadFloatImagePng(
          GetTestDataPath("obstacle_synthetic_depth_golden.png"),
          kDepthImageScale, kDepthImageShift, util::kReverseLandscapeRotation));

  // Clamp the out of range values to match the golden image.
  callback.callback_depth_map->data().forEach<float>(
      [](float& p, const int* position) -> void { p = std::min(p, 65.535f); });

  EXPECT_TRUE(ImagesAlmostEqual(depth_golden, *callback.callback_depth_map));

  GL_ASSERT_OK(detector->Stop());
}

TEST(GuidelineDetector, SegmentationMask) {
  GL_ASSERT_OK_AND_ASSIGN(
      auto image,
      util::LoadImage(GetTestDataPath("guideline1.png"),
                      /*keep_alpha=*/false, util::kReverseLandscapeRotation));
  constexpr uint64_t kImageTimestampNs = 111222333444555;
  image.mutable_metadata().timestamp_ns = kImageTimestampNs;
  auto shared_image = std::make_shared<util::Image>(std::move(image));

  GuidelineEngineConfig_DetectorOptions options;
  GL_ASSERT_OK_AND_ASSIGN(auto detector, GuidelineDetector::Create(options));

  TestDetectionCallback callback;
  detector->AddCallback(
      absl::bind_front(&TestDetectionCallback::OnDetection, &callback));

  GL_ASSERT_OK(detector->Start());

  detector->OnImage(shared_image);

  callback.notification.WaitForNotificationWithTimeout(absl::Seconds(10));

  EXPECT_EQ(callback.callback_timestamp_us, kImageTimestampNs / 1000);
  ASSERT_TRUE(callback.callback_keypoints.has_value());
  ASSERT_TRUE(callback.callback_guideline_mask != nullptr);

  constexpr double kMaskImageScale = 65535.;
  constexpr double kMaskImageShift = 0.;

  // Use the following code to save the golden mask image.
  // GL_ASSERT_OK(util::SaveFloatImagePng(*callback.callback_guideline_mask,
  //                                     "/tmp/guideline1_mask_golden.png",
  //                                     kMaskImageScale, kMaskImageShift,
  //                                     util::kReverseLandscapeRotation));

  GL_ASSERT_OK_AND_ASSIGN(
      auto mask_golden,
      util::LoadFloatImagePng(GetTestDataPath("guideline1_mask_golden.png"),
                              kMaskImageScale, kMaskImageShift,
                              util::kReverseLandscapeRotation));

  EXPECT_TRUE(
      ImagesAlmostEqual(mask_golden, *callback.callback_guideline_mask));

  GL_ASSERT_OK(detector->Stop());
}

TEST(GuidelineDetector, LineKeypoints) {
  GL_ASSERT_OK_AND_ASSIGN(
      auto image,
      util::LoadImage(GetTestDataPath("guideline1.png"),
                      /*keep_alpha=*/false, util::kReverseLandscapeRotation));
  constexpr uint64_t kImageTimestampNs = 111222333444555;
  image.mutable_metadata().timestamp_ns = kImageTimestampNs;
  auto shared_image = std::make_shared<util::Image>(std::move(image));

  GuidelineEngineConfig_DetectorOptions options;
  GL_ASSERT_OK_AND_ASSIGN(auto detector, GuidelineDetector::Create(options));

  TestDetectionCallback callback;
  detector->AddCallback(
      absl::bind_front(&TestDetectionCallback::OnDetection, &callback));

  GL_ASSERT_OK(detector->Start());

  detector->OnImage(shared_image);

  callback.notification.WaitForNotificationWithTimeout(absl::Seconds(10));

  EXPECT_EQ(callback.callback_timestamp_us, kImageTimestampNs / 1000);
  ASSERT_TRUE(callback.callback_keypoints.has_value());

  auto keypoints = *callback.callback_keypoints;
  ASSERT_EQ(keypoints.size(), 204);

  constexpr float kKeypointTolerance = 0.01;
  EXPECT_EIGEN_APPROX(keypoints[0], Eigen::Vector3f(0.69, 0.00, 1.0),
                      kKeypointTolerance);
  EXPECT_EIGEN_APPROX(keypoints[60], Eigen::Vector3f(0.65, 0.25, 1.0),
                      kKeypointTolerance);
  EXPECT_EIGEN_APPROX(keypoints[160], Eigen::Vector3f(0.52, 0.67, 1.0),
                      kKeypointTolerance);
  EXPECT_EIGEN_APPROX(keypoints[200], Eigen::Vector3f(0.28, 0.84, 1.0),
                      kKeypointTolerance);
  EXPECT_EIGEN_APPROX(keypoints[203], Eigen::Vector3f(0.28, 0.85, 0.21),
                      kKeypointTolerance);

  GL_ASSERT_OK(detector->Stop());
}

}  // namespace
}  // namespace guideline::vision
