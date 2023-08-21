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

#include "gtest/gtest.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/calculator_runner.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/matrix.h"
#include "mediapipe/framework/packet.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "project_guideline/camera/cv_camera_model.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/image_io.h"
#include "project_guideline/util/mediapipe.h"
#include "project_guideline/util/transformation.h"

namespace guideline::vision {
namespace {

using mediapipe::CalculatorGraphConfig;
using mediapipe::ParseTextProtoOrDie;

const char kTestDataDir[] =
    "/project_guideline/project_guideline/vision/testdata/";

std::string GetTestDataPath(const std::string& filename) {
  return std::string(getenv("TEST_SRCDIR")) + kTestDataDir + filename;
}

TEST(OccupancyMapCalculator, OccupancyMap) {
  auto calculator_node = ParseTextProtoOrDie<CalculatorGraphConfig::Node>(
      R"pb(
        calculator: "OccupancyMapCalculator"
        input_stream: "DEPTH_IMAGE:input"
        input_stream: "CAMERA_POSE:camera_pose"
        input_stream: "CAMERA_PARAMS:camera_params"
        output_stream: "OCCUPANCY_MAP:output_map"
        output_stream: "OCCUPIED:output_occupied"
        node_options: {
          [type.googleapis.com/
           guideline.vision.OccupancyMapCalculatorOptions]: {
            width_meters: 6
            depth_meters: 10
            bottom_meters: -0.5
            top_meters: 1
            point_confidence_threshold: 0.5
            occupancy_threshold: 3
          }
        }
      )pb");
  mediapipe::CalculatorRunner runner(calculator_node);

  constexpr double kDepthImageScale = 1000.;
  GL_ASSERT_OK_AND_ASSIGN(
      auto input_depth,
      util::LoadFloatImagePng(
          GetTestDataPath("obstacle_synthetic_depth_aligned_golden.png"),
          kDepthImageScale));

  auto input_frame_packet = mediapipe::MakePacket<mediapipe::ImageFrame>(
      util::CopyImageToImageFrame(input_depth));
  runner.MutableInputs()
      ->Tag("DEPTH_IMAGE")
      .packets.push_back(input_frame_packet.At(mediapipe::Timestamp(1)));

  util::Transformation forward_xy =
      util::LookAt({5, 5, 1}, {6, 6, 1}, Eigen::Vector3d::UnitZ());

  mediapipe::Matrix camera_pose;
  camera_pose.resize(1, 7);
  camera_pose.row(0) << 0, 0, 0, forward_xy.q().x(), forward_xy.q().y(),
      forward_xy.q().z(), forward_xy.q().w();

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

  GL_ASSERT_OK(runner.Run());

  const auto& outputs = runner.Outputs();
  EXPECT_EQ(outputs.NumEntries(), 2);
  const auto& occupancy_map =
      outputs.Tag("OCCUPANCY_MAP").packets[0].Get<mediapipe::Matrix>();

  ASSERT_EQ(occupancy_map.rows(), 1);
  ASSERT_EQ(occupancy_map.cols(), 83 * 3);

  ASSERT_TRUE(outputs.Tag("OCCUPIED").packets[0].Get<bool>());
}

}  // namespace
}  // namespace guideline::vision
