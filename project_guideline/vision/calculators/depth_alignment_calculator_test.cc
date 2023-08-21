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

#include <utility>

#include "gtest/gtest.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/calculator_runner.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/formats/matrix.h"
#include "mediapipe/framework/packet.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "project_guideline/camera/cv_camera_model.h"
#include "project_guideline/testing/images.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/image_io.h"
#include "project_guideline/util/mediapipe.h"

namespace guideline::vision {
namespace {

using ::drishti::LandmarkList;
using mediapipe::CalculatorGraphConfig;
using mediapipe::ParseTextProtoOrDie;

const char kTestDataDir[] =
    "/project_guideline/project_guideline/vision/testdata/";

std::string GetTestDataPath(const std::string& filename) {
  return std::string(getenv("TEST_SRCDIR")) + kTestDataDir + filename;
}

void AddLandmark(LandmarkList& list, float x, float y, float z) {
  auto landmark = list.add_landmark();
  landmark->set_x(x);
  landmark->set_y(y);
  landmark->set_z(z);
}

TEST(DepthAlignmentCalculator, AlignDepth) {
  auto calculator_node = ParseTextProtoOrDie<CalculatorGraphConfig::Node>(
      R"pb(
        calculator: "DepthAlignmentCalculator"
        input_stream: "DEPTH_IMAGE:input"
        input_stream: "LANDMARKS:landmarks"
        input_stream: "CAMERA_POSE:camera_pose"
        input_stream: "CAMERA_PARAMS:camera_params"
        output_stream: "DEPTH_IMAGE_ALIGNED:output"
        output_stream: "INV_SCALE_SHIFT:params"
        output_stream: "DEPTH_ALIGNMENT_SUCCESS:success"
        node_options: {
          [type.googleapis.com/
           guideline.vision.DepthAlignmentCalculatorOptions]: {
            input_inverse_depth: true
            output_inverse_depth: false
          }
        }
      )pb");
  mediapipe::CalculatorRunner runner(calculator_node);

  constexpr double kDepthImageScale = 1000.;
  GL_ASSERT_OK_AND_ASSIGN(
      auto input_inv_depth,
      util::LoadFloatImagePng(
          GetTestDataPath("obstacle_synthetic_inverse_depth.png"),
          kDepthImageScale));

  auto input_frame_packet = mediapipe::MakePacket<mediapipe::ImageFrame>(
      util::CopyImageToImageFrame(input_inv_depth));
  runner.MutableInputs()
      ->Tag("DEPTH_IMAGE")
      .packets.push_back(input_frame_packet.At(mediapipe::Timestamp(1)));

  mediapipe::Matrix camera_pose;
  camera_pose.resize(1, 7);
  camera_pose.row(0) << 0, 0, 0, 0, 0, 0, 1;

  runner.MutableInputs()
      ->Tag("CAMERA_POSE")
      .packets.push_back(mediapipe::MakePacket<mediapipe::Matrix>(camera_pose)
                             .At(mediapipe::Timestamp(1)));

  camera::CvCameraModel camera_model(640, 480, {50, 50, 320, 240});
  mediapipe::Matrix camera_params;
  camera_params.resize(1, 6);
  camera_params.row(0) << camera_model.image_width(),
      camera_model.image_height(), camera_model.pinhole_params()[0],
      camera_model.pinhole_params()[1], camera_model.pinhole_params()[2],
      camera_model.pinhole_params()[3];

  runner.MutableInputs()
      ->Tag("CAMERA_PARAMS")
      .packets.push_back(mediapipe::MakePacket<mediapipe::Matrix>(camera_params)
                             .At(mediapipe::Timestamp(1)));

  constexpr float kInvScale = 0.033;
  constexpr float kInvShift = -0.27;
  LandmarkList landmarks;
  for (int x = 0; x < input_inv_depth.width(); x += 10) {
    for (int y = 0; y < input_inv_depth.height(); y += 10) {
      Eigen::Vector3d ray;
      if (camera_model.PixelToRay({x, y}, ray)) {
        double depth = 1.0 / (input_inv_depth.at(y, x) * kInvScale + kInvShift);
        ray = ray * (depth / ray.z());
        AddLandmark(landmarks, ray.x(), ray.y(), ray.z());
      }
    }
  }

  auto input_landmarks_packet =
      mediapipe::MakePacket<LandmarkList>(std::move(landmarks));
  runner.MutableInputs()
      ->Tag("LANDMARKS")
      .packets.push_back(input_landmarks_packet.At(mediapipe::Timestamp(1)));

  GL_ASSERT_OK(runner.Run());

  const auto& outputs = runner.Outputs();
  EXPECT_EQ(outputs.NumEntries(), 3);
  EXPECT_TRUE(outputs.Tag("DEPTH_ALIGNMENT_SUCCESS").packets[0].Get<bool>());

  const auto& output_mask = outputs.Tag("DEPTH_IMAGE_ALIGNED")
                                .packets[0]
                                .Get<mediapipe::ImageFrame>();

  const auto& params =
      outputs.Tag("INV_SCALE_SHIFT").packets[0].Get<std::pair<float, float>>();
  EXPECT_NEAR(params.first, kInvScale, 1e-3);
  EXPECT_NEAR(params.second, kInvShift, 1e-3);

  GL_ASSERT_OK_AND_ASSIGN(
      auto aligned_golden,
      util::LoadFloatImagePng(
          GetTestDataPath("obstacle_synthetic_depth_aligned_golden.png"),
          kDepthImageScale));
  util::Image depth_aligned_image = util::CopyImageFrameToImage(output_mask);

  // Uncomment to update golden image.
  // GL_ASSERT_OK(util::SaveFloatImagePng(depth_aligned_image,
  // "/tmp/obstacle_synthetic_depth_aligned_golden.png",
  //                                     kDepthImageScale));

  EXPECT_TRUE(ImagesAlmostEqual(aligned_golden, depth_aligned_image, 1e-3));
}

TEST(DepthAlignmentCalculator, AlignmentFails) {
  auto calculator_node = ParseTextProtoOrDie<CalculatorGraphConfig::Node>(
      R"pb(
        calculator: "DepthAlignmentCalculator"
        input_stream: "DEPTH_IMAGE:input"
        input_stream: "LANDMARKS:landmarks"
        input_stream: "CAMERA_POSE:camera_pose"
        input_stream: "CAMERA_PARAMS:camera_params"
        output_stream: "DEPTH_IMAGE_ALIGNED:output"
        output_stream: "INV_SCALE_SHIFT:params"
        output_stream: "DEPTH_ALIGNMENT_SUCCESS:success"
        node_options: {
          [type.googleapis.com/
           guideline.vision.DepthAlignmentCalculatorOptions]: {
            input_inverse_depth: true
            output_inverse_depth: false
          }
        }
      )pb");
  mediapipe::CalculatorRunner runner(calculator_node);

  constexpr double kDepthImageScale = 1000.;
  GL_ASSERT_OK_AND_ASSIGN(
      auto input_inv_depth,
      util::LoadFloatImagePng(
          GetTestDataPath("obstacle_synthetic_inverse_depth.png"),
          kDepthImageScale));

  auto input_frame_packet = mediapipe::MakePacket<mediapipe::ImageFrame>(
      util::CopyImageToImageFrame(input_inv_depth));
  runner.MutableInputs()
      ->Tag("DEPTH_IMAGE")
      .packets.push_back(input_frame_packet.At(mediapipe::Timestamp(1)));

  mediapipe::Matrix camera_pose;
  camera_pose.resize(1, 7);
  camera_pose.row(0) << 0, 0, 0, 0, 0, 0, 1;

  runner.MutableInputs()
      ->Tag("CAMERA_POSE")
      .packets.push_back(mediapipe::MakePacket<mediapipe::Matrix>(camera_pose)
                             .At(mediapipe::Timestamp(1)));

  camera::CvCameraModel camera_model(640, 480, {50, 50, 320, 240});
  mediapipe::Matrix camera_params;
  camera_params.resize(1, 6);
  camera_params.row(0) << camera_model.image_width(),
      camera_model.image_height(), camera_model.pinhole_params()[0],
      camera_model.pinhole_params()[1], camera_model.pinhole_params()[2],
      camera_model.pinhole_params()[3];

  runner.MutableInputs()
      ->Tag("CAMERA_PARAMS")
      .packets.push_back(mediapipe::MakePacket<mediapipe::Matrix>(camera_params)
                             .At(mediapipe::Timestamp(1)));

  LandmarkList landmarks;
  landmarks.add_landmark()->set_x(100.0);

  auto input_landmarks_packet =
      mediapipe::MakePacket<LandmarkList>(std::move(landmarks));
  runner.MutableInputs()
      ->Tag("LANDMARKS")
      .packets.push_back(input_landmarks_packet.At(mediapipe::Timestamp(1)));

  EXPECT_FALSE(runner.Run().ok());
}

TEST(DepthAlignmentCalculator, AlignmentFailsWithFallback) {
  auto calculator_node = ParseTextProtoOrDie<CalculatorGraphConfig::Node>(
      R"pb(
        calculator: "DepthAlignmentCalculator"
        input_stream: "DEPTH_IMAGE:input"
        input_stream: "LANDMARKS:landmarks"
        input_stream: "CAMERA_POSE:camera_pose"
        input_stream: "CAMERA_PARAMS:camera_params"
        input_stream: "FALLBACK_INV_SCALE_SHIFT:fallback_scale_shift"
        output_stream: "DEPTH_IMAGE_ALIGNED:output"
        output_stream: "INV_SCALE_SHIFT:params"
        output_stream: "DEPTH_ALIGNMENT_SUCCESS:success"
        node_options: {
          [type.googleapis.com/
           guideline.vision.DepthAlignmentCalculatorOptions]: {
            input_inverse_depth: true
            output_inverse_depth: false
          }
        }
      )pb");
  mediapipe::CalculatorRunner runner(calculator_node);

  constexpr double kDepthImageScale = 1000.;
  GL_ASSERT_OK_AND_ASSIGN(
      auto input_inv_depth,
      util::LoadFloatImagePng(
          GetTestDataPath("obstacle_synthetic_inverse_depth.png"),
          kDepthImageScale));

  auto input_frame_packet = mediapipe::MakePacket<mediapipe::ImageFrame>(
      util::CopyImageToImageFrame(input_inv_depth));
  runner.MutableInputs()
      ->Tag("DEPTH_IMAGE")
      .packets.push_back(input_frame_packet.At(mediapipe::Timestamp(1)));

  mediapipe::Matrix camera_pose;
  camera_pose.resize(1, 7);
  camera_pose.row(0) << 0, 0, 0, 0, 0, 0, 1;

  runner.MutableInputs()
      ->Tag("CAMERA_POSE")
      .packets.push_back(mediapipe::MakePacket<mediapipe::Matrix>(camera_pose)
                             .At(mediapipe::Timestamp(1)));

  camera::CvCameraModel camera_model(640, 480, {50, 50, 320, 240});
  mediapipe::Matrix camera_params;
  camera_params.resize(1, 6);
  camera_params.row(0) << camera_model.image_width(),
      camera_model.image_height(), camera_model.pinhole_params()[0],
      camera_model.pinhole_params()[1], camera_model.pinhole_params()[2],
      camera_model.pinhole_params()[3];

  runner.MutableInputs()
      ->Tag("CAMERA_PARAMS")
      .packets.push_back(mediapipe::MakePacket<mediapipe::Matrix>(camera_params)
                             .At(mediapipe::Timestamp(1)));

  constexpr float kInvScale = 0.033;
  constexpr float kInvShift = -0.27;
  runner.MutableInputs()
      ->Tag("FALLBACK_INV_SCALE_SHIFT")
      .packets.push_back(
          mediapipe::MakePacket<std::pair<float, float>>(kInvScale, kInvShift)
              .At(mediapipe::Timestamp(1)));

  LandmarkList landmarks;
  landmarks.add_landmark()->set_x(100.0);

  auto input_landmarks_packet =
      mediapipe::MakePacket<LandmarkList>(std::move(landmarks));
  runner.MutableInputs()
      ->Tag("LANDMARKS")
      .packets.push_back(input_landmarks_packet.At(mediapipe::Timestamp(1)));

  GL_ASSERT_OK(runner.Run());

  const auto& outputs = runner.Outputs();
  EXPECT_EQ(outputs.NumEntries(), 3);
  EXPECT_FALSE(outputs.Tag("DEPTH_ALIGNMENT_SUCCESS").packets[0].Get<bool>());

  const auto& params =
      outputs.Tag("INV_SCALE_SHIFT").packets[0].Get<std::pair<float, float>>();
  EXPECT_NEAR(params.first, kInvScale, 1e-3);
  EXPECT_NEAR(params.second, kInvShift, 1e-3);
}

}  // namespace
}  // namespace guideline::vision
