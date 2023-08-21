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

#include "project_guideline/environment/guidance_system.h"

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "google/protobuf/duration.pb.h"
#include "gtest/gtest.h"
#include <opencv2/core/mat.hpp>
#include "absl/flags/flag.h"
#include "absl/status/status.h"
#include "project_guideline/camera/cv_camera_model.h"
#include "project_guideline/environment/control_signal.h"
#include "project_guideline/environment/environment.h"
#include "project_guideline/environment/human_representation.h"
#include "project_guideline/environment/path_planning.h"
#include "project_guideline/logging/file_guideline_logger.h"
#include "project_guideline/motion/tracking_feature.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/status.h"
#include "project_guideline/util/transformation.h"

namespace guideline::environment {

namespace {

using ::Eigen::Vector3d;
using ::Eigen::Vector3f;
using environment::CameraPoseBasedHuman;
using environment::Environment;
using environment::OccupancyMap;
using environment::PointCloud;
using environment::SimpleControlSystem;
using environment::SortedGuidelineBoxPointAggregator;
using logging::FileGuidelineLogger;
using motion::TrackingFeature;
using util::Transformation;

constexpr int kDepthMapWidth = 32;
constexpr int kDepthMapHeight = 24;

static const std::shared_ptr<camera::CvCameraModel> kCameraModel =
    std::make_unique<camera::CvCameraModel>(
        640, 480,
        Eigen::Vector4d(506.44221032264841, 507.0093, 321.6719, 240.1555),
        Eigen::Vector4d(0.085878, -0.19043, 0.12106, 0.0));

float Randomize(float x, float max_dev, uint32_t* seed) {
  float f =
      (2.0f * static_cast<float>(rand_r(seed)) / static_cast<float>(RAND_MAX)) -
      1.0f;
  return x + (f * max_dev);
}

cv::Mat CreateRandomDepthMat(const int width, const int height, float min,
                             float max, uint32_t* seed) {
  cv::Mat mat(height, width, CV_32FC1);
  const float dev = (min - max) / 2.0;
  for (auto iter = mat.begin<float>(); iter != mat.end<float>(); ++iter) {
    *iter = Randomize(min + dev, dev, seed);
  }
  return mat;
}

util::DepthImage CreateDepthImage(int width, int height, float values) {
  cv::Mat depth_mat(height, width, CV_32FC1);
  depth_mat = values;
  return util::DepthImage(width, height,
                          std::make_unique<cv::Mat>(std::move(depth_mat)));
}

util::DepthImageU16 CreateArDepthImage(int width, int height, uint16_t values) {
  cv::Mat depth_mat(height, width, CV_16UC1);
  depth_mat = values;
  return util::DepthImageU16(width, height,
                             std::make_unique<cv::Mat>(std::move(depth_mat)));
}

util::ConfidenceImageU8 CreateArConfidenceImage(int width, int height,
                                                uint8_t values) {
  cv::Mat confidence_mat(height, width, CV_8UC1);
  confidence_mat = values;
  return util::ConfidenceImageU8(
      width, height, std::make_unique<cv::Mat>(std::move(confidence_mat)));
}

util::ConfidenceMask CreateEmptyConfidenceMask() {
  constexpr size_t kConfidenceMaskWidth = 64;
  constexpr size_t kConfidenceMaskHeight = 64;
  cv::Mat confidence_mat(kConfidenceMaskHeight, kConfidenceMaskWidth, CV_32FC1);
  return util::ConfidenceMask(
      kConfidenceMaskWidth, kConfidenceMaskHeight,
      std::make_unique<cv::Mat>(std::move(confidence_mat)));
}

std::vector<TrackingFeature> TrackingFeaturesFromDepthMap(
    const util::DepthImage& depth_map, float scale, float shift) {
  std::vector<TrackingFeature> tracking_features;
  for (int x = 0; x < kDepthMapWidth; x += 4) {
    for (int y = 0; y < kDepthMapHeight; y += 4) {
      float feature_depth = 1.0 / ((1.0 / depth_map(y, x)) * scale + shift);

      Vector3d ray;
      kCameraModel->PixelToRay(
          {1.0 * x * kCameraModel->image_width() / kDepthMapWidth,
           1.0 * y * kCameraModel->image_height() / kDepthMapHeight},
          ray);
      Vector3d camera_t_feature = (feature_depth / ray.z()) * ray;
      tracking_features.emplace_back(camera_t_feature.cast<float>(), 1.0f, 0);
    }
  }
  return tracking_features;
}

std::unique_ptr<GuidanceSystem> InstantiateGuidanceSystem(
    const GuidanceSystemOptions& guidance_system_options) {
  auto tmp_output_dir = std::string(getenv("TEST_TMPDIR"));

  std::shared_ptr<FileGuidelineLogger> file_logger =
      FileGuidelineLogger::Create(tmp_output_dir).value();

  HumanRepresentationOptions human_representation_options;
  GL_CHECK_OK_AND_ASSIGN(
      auto human, CameraPoseBasedHuman::Create(human_representation_options));
  GuidelineAggregatorOptions guideline_aggregator_options;
  GL_CHECK_OK_AND_ASSIGN(
      auto guideline_aggregator,
      SortedGuidelineBoxPointAggregator::Create(guideline_aggregator_options));
  PointCloudOptions point_cloud_options;
  auto point_cloud = PointCloud::Create(point_cloud_options);
  OccupancyMapOptions occupancy_map_options;
  auto occupancy_map = OccupancyMap::Create(occupancy_map_options);
  GL_CHECK_OK_AND_ASSIGN(
      auto environment,
      Environment::Create(std::move(human), std::move(guideline_aggregator),
                          std::move(point_cloud), std::move(occupancy_map),
                          nullptr));

  auto camera_model = std::make_shared<camera::CvCameraModel>(
      640, 480,
      Eigen::Vector4d(506.44221032264841, 507.0093, 321.6719, 240.1555),
      Eigen::Vector4d(0.085878, -0.19043, 0.12106, 0.0));

  ControlSystemOptions control_system_options;
  control_system_options.mutable_simple_control_system_options()
      ->set_lateral_movement_abs_max_threshold_meters(4);
  GL_CHECK_OK_AND_ASSIGN(auto control_system,
                         SimpleControlSystem::Create(control_system_options));

  GL_CHECK_OK_AND_ASSIGN(
      auto guidance_system,
      GuidanceSystem::Create(guidance_system_options, file_logger,
                             std::move(environment),
                             std::move(control_system)));

  return guidance_system;
}

TEST(GuidanceSystemTest, TestCreation) {
  InstantiateGuidanceSystem(GuidanceSystemOptions());
}

TEST(GuidanceSystemTest, ControlSignals) {
  auto guidance_system = InstantiateGuidanceSystem(GuidanceSystemOptions());

  std::optional<environment::ControlSignal> signal;
  const ControlSignalCallback callback =
      [&signal](const environment::ControlSignal& control_signal) {
        signal = control_signal;
      };

  guidance_system->AddControlSignalCallback(callback);

  // Create keypoints for line which extends up center of screen. Note the
  // aggregator expects these are in order from furthest to nearest (due to
  // reverse landscape orientation and the image being upside-down).
  const int kNumKeypoints = 16;
  std::vector<Eigen::Vector3f> keypoints;
  for (int i = kNumKeypoints - 1; i > 0; --i) {
    Eigen::Vector3f keypoint(0.5, 1.0f * i / kNumKeypoints, 1.0);
    keypoints.emplace_back(keypoint);
  }

  Transformation t =
      util::LookAt({5, 0, 0}, {5, 1, 0}, Eigen::Vector3d::UnitZ());

  int timestamp = 0;
  for (int i = 0; i < 10; i++) {
    guidance_system->OnCameraPose(++timestamp, t, kCameraModel);
    guidance_system->OnDetection(
        timestamp, keypoints, CreateEmptyConfidenceMask(),
        util::DepthImage(kDepthMapWidth, kDepthMapHeight,
                         std::make_unique<cv::Mat>(kDepthMapHeight,
                                                   kDepthMapWidth, CV_32FC1)));
  }

  EXPECT_TRUE(signal.has_value());
  EXPECT_NEAR(signal->lateral_movement_meters, 0., 1e-2);
  EXPECT_NEAR(signal->rotation_movement_degrees, 0., 1);
  EXPECT_NEAR(signal->target_rotation_movement_degrees, 0, 1);
  EXPECT_FALSE(signal->stop);

  // Move 5m ahead and 1.5m to the right of the line.
  t = Transformation(t.q(), t.p() + Vector3d(1.5, 5., 0.));
  guidance_system->OnCameraPose(++timestamp, t, kCameraModel);
  EXPECT_NEAR(signal->lateral_movement_meters, -1.5, 0.05);
  EXPECT_NEAR(signal->rotation_movement_degrees, 0., 1);
  EXPECT_NEAR(signal->target_rotation_movement_degrees, -27, 1);

  // Change the orientation of the camera, 45 degrees to the right and downward.
  t = util::LookAt(t.p(), t.p() + Eigen::Vector3d(1, 1, -1),
                   Eigen::Vector3d::UnitZ());

  // These keypoints don't properly correspond to the camera orientation, so
  // the results of the hit test are spurious and will affect the guideline.
  keypoints.clear();
  for (int i = kNumKeypoints - 1; i > 0; i--) {
    Eigen::Vector3f keypoint(0.1, 1.0f * i / kNumKeypoints, 1.0);
    keypoints.emplace_back(keypoint);
  }

  for (int i = 0; i < 10; i++) {
    guidance_system->OnCameraPose(++timestamp, t, kCameraModel);
    guidance_system->OnDetection(
        timestamp, keypoints, CreateEmptyConfidenceMask(),
        CreateDepthImage(kDepthMapWidth, kDepthMapHeight, 0));
  }

  EXPECT_NEAR(signal->lateral_movement_meters, -0.24, 0.05);
  EXPECT_NEAR(signal->rotation_movement_degrees, 30, 1);
  EXPECT_NEAR(signal->target_rotation_movement_degrees, -55, 1);
  EXPECT_FALSE(signal->stop);
}

TEST(GuidanceSystemTest, TrackingStopSignal) {
  auto guidance_system = InstantiateGuidanceSystem(GuidanceSystemOptions());

  std::optional<environment::ControlSignal> signal;
  const ControlSignalCallback callback =
      [&signal](const environment::ControlSignal& control_signal) {
        signal = control_signal;
      };

  guidance_system->AddControlSignalCallback(callback);

  const int kNumKeypoints = 16;
  std::vector<Eigen::Vector3f> keypoints;
  for (int i = 0; i < kNumKeypoints; i++) {
    Eigen::Vector3f keypoint(0.5, i / (float)kNumKeypoints, 1.0);
    keypoints.emplace_back(keypoint);
  }

  Transformation t =
      util::LookAt({0, 0, 0}, {1, 1, -1}, Eigen::Vector3d::UnitZ());
  for (int i = 0; i < 10; i++) {
    guidance_system->OnCameraPose(/*timestamp_us=*/i, t, kCameraModel);
    guidance_system->OnDetection(
        /*timestamp_us=*/i, keypoints, CreateEmptyConfidenceMask(),
        CreateDepthImage(kDepthMapWidth, kDepthMapHeight, 0));
  }

  guidance_system->OnTrackingStateChanged(false);
  ASSERT_TRUE(signal->stop);
}

TEST(GuidanceSystemTest, EagerStopWithoutOption) {
  auto guidance_system = InstantiateGuidanceSystem(GuidanceSystemOptions());
  std::optional<environment::ControlSignal> signal;
  const ControlSignalCallback callback =
      [&signal](const environment::ControlSignal& control_signal) {
        signal = control_signal;
      };

  guidance_system->AddControlSignalCallback(callback);
  Transformation t =
      util::LookAt({0, 0, 0}, {1, 1, -1}, Eigen::Vector3d::UnitZ());
  std::vector<Eigen::Vector3f> emptyKeypoints;

  util::DepthImage depth_map =
      CreateDepthImage(kDepthMapWidth, kDepthMapHeight, 0);
  guidance_system->OnCameraPose(/*timestamp_us=*/0, t, kCameraModel);
  guidance_system->OnDetection(/*timestamp_us=*/0, emptyKeypoints,
                               CreateEmptyConfidenceMask(), depth_map);
  ASSERT_FALSE(signal.has_value());
  guidance_system->OnCameraPose(/*timestamp_us=*/2000000, t, kCameraModel);
  guidance_system->OnDetection(/*timestamp_us=*/2000000, emptyKeypoints,
                               CreateEmptyConfidenceMask(), depth_map);
  ASSERT_FALSE(signal.has_value());
}

TEST(GuidanceSystemTest, EmptyKeypointsStopSignal) {
  auto guidance_system_options = GuidanceSystemOptions();
  guidance_system_options.mutable_eager_stop_threshold()->set_seconds(1);
  auto guidance_system = InstantiateGuidanceSystem(guidance_system_options);

  std::optional<environment::ControlSignal> signal;
  const ControlSignalCallback callback =
      [&signal](const environment::ControlSignal& control_signal) {
        signal = control_signal;
      };

  guidance_system->AddControlSignalCallback(callback);
  Transformation t =
      util::LookAt({0, 0, 0}, {1, 1, -1}, Eigen::Vector3d::UnitZ());

  std::vector<Eigen::Vector3f> emptyKeypoints;
  std::vector<Eigen::Vector3f> NonEmptyKeypoints;
  NonEmptyKeypoints.emplace_back(0.5, 0.5, 1);

  util::DepthImage depth_map =
      CreateDepthImage(kDepthMapWidth, kDepthMapHeight, 0);
  guidance_system->OnCameraPose(/*timestamp_us=*/0, t, kCameraModel);
  guidance_system->OnDetection(/*timestamp_us=*/0, emptyKeypoints,
                               CreateEmptyConfidenceMask(), depth_map);
  ASSERT_FALSE(signal.has_value());
  guidance_system->OnCameraPose(/*timestamp_us=*/500000, t, kCameraModel);
  guidance_system->OnDetection(/*timestamp_us=*/500000, emptyKeypoints,
                               CreateEmptyConfidenceMask(), depth_map);
  ASSERT_FALSE(signal.has_value());
  guidance_system->OnCameraPose(/*timestamp_us=*/1000000, t, kCameraModel);
  guidance_system->OnDetection(/*timestamp_us=*/1000000, NonEmptyKeypoints,
                               CreateEmptyConfidenceMask(), depth_map);
  ASSERT_FALSE(signal.has_value());
  guidance_system->OnCameraPose(/*timestamp_us=*/1500000, t, kCameraModel);
  guidance_system->OnDetection(/*timestamp_us=*/1500000, emptyKeypoints,
                               CreateEmptyConfidenceMask(), depth_map);
  ASSERT_FALSE(signal.has_value());
  guidance_system->OnCameraPose(/*timestamp_us=*/2000000, t, kCameraModel);
  guidance_system->OnDetection(/*timestamp_us=*/2000000, emptyKeypoints,
                               CreateEmptyConfidenceMask(), depth_map);
  ASSERT_FALSE(signal.has_value());
  guidance_system->OnCameraPose(/*timestamp_us=*/2500000, t, kCameraModel);
  guidance_system->OnDetection(/*timestamp_us=*/2500000, emptyKeypoints,
                               CreateEmptyConfidenceMask(), depth_map);
  ASSERT_TRUE(signal->stop);
}

TEST(GuidanceSystemTest, ObstacleDetectionArDepth) {
  GuidanceSystemOptions guidance_system_options;
  guidance_system_options.set_enable_obstacle_detection(true);
  auto guidance_system = InstantiateGuidanceSystem(guidance_system_options);

  // Makes all extrinsic transforms the identity.
  Transformation look_forward = util::LookAt(
      /*src=*/Vector3d::Zero(),
      /*look_at=*/Vector3d::UnitY(),
      /*up=*/Vector3d::UnitZ());

  // Sets depth map as no obstacles exist in clearance zone.
  constexpr uint16_t kDepthBackgroundMm = 20 * 1000;
  auto depth_image =
      CreateArDepthImage(kDepthMapWidth, kDepthMapHeight, kDepthBackgroundMm);

  constexpr uint8_t kConfidenceUint8 = 100;
  auto confidence_image = CreateArConfidenceImage(
      kDepthMapWidth, kDepthMapHeight, kConfidenceUint8);

  std::optional<environment::ControlSignal> signal;
  const ControlSignalCallback callback =
      [&signal](const environment::ControlSignal& control_signal) {
        signal = control_signal;
      };

  guidance_system->AddControlSignalCallback(callback);

  const int kNumKeypoints = 16;
  std::vector<Eigen::Vector3f> keypoints;
  for (int i = 0; i < kNumKeypoints; ++i) {
    Eigen::Vector3f keypoint(0.5, i / (float)kNumKeypoints, 1.0);
    keypoints.emplace_back(keypoint);
  }

  for (int i = 0; i < 10; ++i) {
    guidance_system->OnCameraPose(/*timestamp_us=*/i, look_forward,
                                  kCameraModel);
    guidance_system->OnArDepth(i, depth_image, confidence_image);
    guidance_system->OnDetection(
        /*timestamp_us=*/i, keypoints, CreateEmptyConfidenceMask(),
        CreateDepthImage(kDepthMapWidth, kDepthMapHeight, 0));
  }
  ASSERT_FALSE(signal->stop);
  ASSERT_FALSE(signal->obstacle_ahead);

  // Sets depth map as the obstacles exist in clearance zone.
  constexpr uint16_t kDepthForegroundMm = 2 * 1000;
  auto depth_image2 =
      CreateArDepthImage(kDepthMapWidth, kDepthMapHeight, kDepthForegroundMm);

  for (int i = 10; i < 30; ++i) {
    guidance_system->OnCameraPose(/*timestamp_us=*/i, look_forward,
                                  kCameraModel);
    guidance_system->OnArDepth(/*timestamp_us=*/i, depth_image2,
                               confidence_image);
  }

  // Checks the 'obstacle_ahead' signal is generated.
  ASSERT_FALSE(signal->stop);
  ASSERT_TRUE(signal->obstacle_ahead);
  ASSERT_TRUE(guidance_system->GetGuideline().status().ok());
}

TEST(GuidanceSystemTest, ObstacleDetectionDepthMap) {
  GuidanceSystemOptions guidance_system_options;
  guidance_system_options.set_enable_obstacle_detection(true);
  guidance_system_options.set_align_depth_with_features(false);
  auto guidance_system = InstantiateGuidanceSystem(guidance_system_options);

  // Makes all extrinsic transforms the identity.
  Transformation look_forward = util::LookAt(
      /*src=*/Vector3d::Zero(),
      /*look_at=*/Vector3d::UnitY(),
      /*up=*/Vector3d::UnitZ());

  // Sets depth map as no obstacles exist in clearance zone.
  constexpr float kBackgroundDepthMeters = 100;
  cv::Mat depth_mat(kDepthMapHeight, kDepthMapWidth, CV_32FC1);
  depth_mat = kBackgroundDepthMeters;

  std::optional<environment::ControlSignal> signal;
  const ControlSignalCallback callback =
      [&signal](const environment::ControlSignal& control_signal) {
        signal = control_signal;
      };

  guidance_system->AddControlSignalCallback(callback);

  const int kNumKeypoints = 16;
  std::vector<Eigen::Vector3f> keypoints;
  for (int i = 0; i < kNumKeypoints; ++i) {
    Eigen::Vector3f keypoint(0.5, i / (float)kNumKeypoints, 1.0);
    keypoints.emplace_back(keypoint);
  }

  util::DepthImage depth_map(kDepthMapWidth, kDepthMapHeight,
                             std::make_unique<cv::Mat>(depth_mat.clone()));
  for (int i = 0; i < 10; ++i) {
    guidance_system->OnCameraPose(/*timestamp_us=*/i, look_forward,
                                  kCameraModel);
    guidance_system->OnDetection(/*timestamp_us=*/i, keypoints,
                                 CreateEmptyConfidenceMask(), depth_map);
  }
  ASSERT_FALSE(signal->stop);
  ASSERT_FALSE(signal->obstacle_ahead);

  // Sets depth map as the obstacles exist in clearance zone.
  constexpr int kObstacleSize = 4;
  constexpr int kObstacleDepthMeters = 2;
  for (int y = kDepthMapHeight / 2 - kObstacleSize / 2;
       y < kDepthMapHeight / 2 + kObstacleSize / 2; ++y) {
    for (int x = kDepthMapWidth / 2 - kObstacleSize / 2;
         x < kDepthMapWidth / 2 + kObstacleSize / 2; ++x) {
      depth_mat.at<float>(y, x) = kObstacleDepthMeters;
    }
  }

  util::DepthImage depth_map2 =
      util::DepthImage(kDepthMapWidth, kDepthMapHeight,
                       std::make_unique<cv::Mat>(depth_mat.clone()));
  for (int i = 10; i < 30; ++i) {
    guidance_system->OnCameraPose(/*timestamp_us=*/i, look_forward,
                                  kCameraModel);
    guidance_system->OnDetection(/*timestamp_us=*/i, keypoints,
                                 CreateEmptyConfidenceMask(), depth_map2);
  }

  // Checks the 'obstacle_ahead' signal is generated.
  ASSERT_FALSE(signal->stop);
  ASSERT_TRUE(signal->obstacle_ahead);
  ASSERT_TRUE(guidance_system->GetGuideline().status().ok());
}

TEST(GuidanceSystemTest, ObstacleDetectionDepthMapWithAlignment) {
  GuidanceSystemOptions guidance_system_options;
  guidance_system_options.set_enable_obstacle_detection(true);
  guidance_system_options.set_align_depth_with_features(true);
  auto guidance_system = InstantiateGuidanceSystem(guidance_system_options);

  // Makes all extrinsic transforms the identity.
  Transformation look_forward = util::LookAt(
      /*src=*/Vector3d::Zero(),
      /*look_at=*/Vector3d::UnitY(),
      /*up=*/Vector3d::UnitZ());

  // Sets depth map as no obstacles exist in clearance zone.
  constexpr int kMinBackgroundDepthMeters = 20;
  constexpr int kMaxBackgroundDepthMeters = 30;
  uint32_t seed = 0;
  cv::Mat depth_mat = CreateRandomDepthMat(kDepthMapWidth, kDepthMapHeight,
                                           kMinBackgroundDepthMeters,
                                           kMaxBackgroundDepthMeters, &seed);
  util::DepthImage depth_map(kDepthMapWidth, kDepthMapHeight,
                             std::make_unique<cv::Mat>(depth_mat.clone()));

  std::optional<environment::ControlSignal> signal;
  const ControlSignalCallback callback =
      [&signal](const environment::ControlSignal& control_signal) {
        signal = control_signal;
      };
  guidance_system->AddControlSignalCallback(callback);

  const int kNumKeypoints = 16;
  std::vector<Eigen::Vector3f> keypoints;
  for (int i = 0; i < kNumKeypoints; ++i) {
    Eigen::Vector3f keypoint(0.5, i / (float)kNumKeypoints, 1.0);
    keypoints.emplace_back(keypoint);
  }

  constexpr float kDepthScale = 0.789;
  constexpr float kDepthShift = 0.0123;
  std::vector<TrackingFeature> tracking_features =
      TrackingFeaturesFromDepthMap(depth_map, kDepthScale, kDepthShift);

  for (int i = 0; i < 10; ++i) {
    guidance_system->OnCameraPose(/*timestamp_us=*/i, look_forward,
                                  kCameraModel);
    guidance_system->OnTrackingFeatures(/*timestamp_us=*/i, tracking_features);
    guidance_system->OnDetection(/*timestamp_us=*/i, keypoints,
                                 CreateEmptyConfidenceMask(), depth_map);
  }
  ASSERT_TRUE(signal.has_value());
  ASSERT_FALSE(signal->stop);
  ASSERT_FALSE(signal->obstacle_ahead);

  // Sets depth map as the obstacles exist in clearance zone.
  constexpr int kObstacleSize = 8;
  constexpr int kObstacleDepthMeters = 2;
  constexpr float kObstacleDepthVariationMeters = 0.01;
  for (int y = kDepthMapHeight / 2 - kObstacleSize / 2;
       y < kDepthMapHeight / 2 + kObstacleSize / 2; ++y) {
    for (int x = kDepthMapWidth / 2 - kObstacleSize / 2;
         x < kDepthMapWidth / 2 + kObstacleSize / 2; ++x) {
      depth_mat.at<float>(y, x) =
          Randomize(kObstacleDepthMeters, kObstacleDepthVariationMeters, &seed);
    }
  }

  util::DepthImage depth_map2(kDepthMapWidth, kDepthMapHeight,
                              std::make_unique<cv::Mat>(depth_mat.clone()));
  tracking_features =
      TrackingFeaturesFromDepthMap(depth_map2, kDepthScale, kDepthShift);

  for (int i = 10; i < 30; ++i) {
    guidance_system->OnCameraPose(/*timestamp_us=*/i, look_forward,
                                  kCameraModel);
    guidance_system->OnTrackingFeatures(/*timestamp_us=*/i, tracking_features);
    guidance_system->OnDetection(/*timestamp_us=*/i, keypoints,
                                 CreateEmptyConfidenceMask(), depth_map2);
  }

  // Checks the 'obstacle_ahead' signal is generated.
  ASSERT_FALSE(signal->stop);
  ASSERT_TRUE(signal->obstacle_ahead);
  ASSERT_TRUE(guidance_system->GetGuideline().status().ok());
}

}  // namespace
}  // namespace guideline::environment
