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

#include <string>
#include <utility>
#include "gtest/gtest.h"
#include "absl/flags/flag.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/calculator_runner.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/formats/matrix.h"
#include "mediapipe/framework/packet.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "project_guideline/camera/cv_camera_model.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/image_io.h"
#include "project_guideline/util/mediapipe.h"
#include "project_guideline/util/transformation.h"

namespace guideline::vision {
namespace {

using ::drishti::LandmarkList;
using mediapipe::CalculatorGraphConfig;
using mediapipe::ParseTextProtoOrDie;

const char kTestDataDir[] =
    "/project_guideline/project_guideline/vision/testdata/";

const char kModelDir[] =
    "/project_guideline/project_guideline/vision/models/";

std::string GetTestDataPath(const std::string& filename) {
  return std::string(getenv("TEST_SRCDIR")) + kTestDataDir + filename;
}

std::string GetModelPath(const std::string& filename) {
  return std::string(getenv("TEST_SRCDIR")) + kModelDir + filename;
}

void AddLandmark(LandmarkList& list, float x, float y, float z) {
  auto landmark = list.add_landmark();
  landmark->set_x(x);
  landmark->set_y(y);
  landmark->set_z(z);
}

TEST(ObstacleDetectionTest, DetectObstacles) {
  auto graph_config =
      ParseTextProtoOrDie<CalculatorGraphConfig>(absl::Substitute(
          R"pb(
            input_stream: "image_frame"
            input_stream: "landmarks"
            input_stream: "camera_pose"
            input_stream: "camera_params"
            output_stream: "obstacle_present"

            node {
              calculator: "ImageSegmentationCalculator"
              input_stream: "IMAGE:image_frame"
              output_stream: "MASK:depth_image"
              node_options: {
                [type.googleapis.com/
                 guideline.vision.ImageSegmentationCalculatorOptions]: {
                  model_file: { file_path: "$0" }
                }
              }
            }

            node {
              calculator: "DepthAlignmentCalculator"
              input_stream: "DEPTH_IMAGE:depth_image"
              input_stream: "LANDMARKS:landmarks"
              input_stream: "CAMERA_POSE:camera_pose"
              input_stream: "CAMERA_PARAMS:camera_params"
              output_stream: "DEPTH_IMAGE_ALIGNED:depth_image_aligned"
              output_stream: "INV_SCALE_SHIFT:depth_params"
              node_options: {
                [type.googleapis.com/
                 guideline.vision.DepthAlignmentCalculatorOptions]: {
                  input_inverse_depth: true
                  output_inverse_depth: false
                }
              }
            }

            node {
              calculator: "OccupancyMapCalculator"
              input_stream: "DEPTH_IMAGE:depth_image_aligned"
              input_stream: "CAMERA_POSE:camera_pose"
              input_stream: "CAMERA_PARAMS:camera_params"
              output_stream: "OCCUPANCY_MAP:occupancy_map"
              output_stream: "OCCUPIED:obstacle_present"
              node_options: {
                [type.googleapis.com/
                 guideline.vision.OccupancyMapCalculatorOptions]: {
                  width_meters: 4
                  depth_meters: 5
                  bottom_meters: -0.5
                  top_meters: 1
                  point_confidence_threshold: 0.5
                  occupancy_threshold: 3
                }
              }
            }
          )pb",
          GetModelPath("depth.tflite")));

  mediapipe::CalculatorGraph graph;
  GL_ASSERT_OK(graph.Initialize(graph_config));
  graph.SetGraphInputStreamAddMode(
      mediapipe::CalculatorGraph::GraphInputStreamAddMode::ADD_IF_NOT_FULL);
  GL_ASSERT_OK_AND_ASSIGN(mediapipe::OutputStreamPoller poller,
                          graph.AddOutputStreamPoller("obstacle_present"));
  GL_ASSERT_OK(graph.StartRun({}));

  // Try first image which has camera pointing in +Y direction with obstacle
  // present. Landmarks are initialized to align with depth image.
  mediapipe::Timestamp t1(1000);

  GL_ASSERT_OK_AND_ASSIGN(
      auto input_frame,
      util::LoadImageFrame(GetTestDataPath("obstacle_synthetic.png")));

  util::Transformation forward_y =
      util::LookAt({0, 0, 0}, {0, 1, 0}, Eigen::Vector3d::UnitZ());

  mediapipe::Matrix camera_pose1;
  camera_pose1.resize(1, 7);
  camera_pose1.row(0) << 0, 0, 0, forward_y.q().x(), forward_y.q().y(),
      forward_y.q().z(), forward_y.q().w();

  GL_ASSERT_OK(graph.AddPacketToInputStream(
      "camera_pose",
      mediapipe::MakePacket<mediapipe::Matrix>(camera_pose1).At(t1)));

  camera::CvCameraModel camera_model(640, 480, {50, 50, 320, 240});
  mediapipe::Matrix camera_params;
  camera_params.resize(1, 6);
  camera_params.row(0) << camera_model.image_width(),
      camera_model.image_height(), camera_model.pinhole_params()[0],
      camera_model.pinhole_params()[1], camera_model.pinhole_params()[2],
      camera_model.pinhole_params()[3];

  GL_ASSERT_OK(graph.AddPacketToInputStream(
      "camera_params",
      mediapipe::MakePacket<mediapipe::Matrix>(camera_params).At(t1)));

  constexpr double kDepthImageScale = 1000.;
  GL_ASSERT_OK_AND_ASSIGN(
      auto input_inv_depth,
      util::LoadFloatImagePng(
          GetTestDataPath("obstacle_synthetic_inverse_depth.png"),
          kDepthImageScale));

  constexpr float kInvScale = 0.033;
  constexpr float kInvShift = -0.27;
  LandmarkList landmarks;
  for (int x = 0; x < input_frame.Width(); x += 10) {
    for (int y = 0; y < input_frame.Height(); y += 10) {
      Eigen::Vector3d ray;
      ASSERT_TRUE(camera_model.PixelToRay({x, y}, ray));
      double depth = 1.0 / (input_inv_depth.at(y, x) * kInvScale + kInvShift);
      ray = ray * (depth / ray.z());
      AddLandmark(landmarks, ray.x(), ray.y(), ray.z());
    }
  }
  auto input_landmarks_packet =
      mediapipe::MakePacket<LandmarkList>(std::move(landmarks));
  GL_ASSERT_OK(
      graph.AddPacketToInputStream("landmarks", input_landmarks_packet.At(t1)));

  auto input_frame_packet =
      mediapipe::MakePacket<mediapipe::ImageFrame>(std::move(input_frame));
  GL_ASSERT_OK(
      graph.AddPacketToInputStream("image_frame", input_frame_packet.At(t1)));

  mediapipe::Packet output1;
  ASSERT_TRUE(poller.Next(&output1));
  ASSERT_TRUE(output1.At(t1).Get<bool>());

  // Add another image with obstacle present, but change the camera pose to
  // point in +X direction. The landmarks are not updated so previous
  // scale/shift depth alignment values are used.
  mediapipe::Timestamp t2(2000);

  util::Transformation forward_x =
      util::LookAt({0, 0, 0}, {1, 0, 0}, Eigen::Vector3d::UnitZ());

  mediapipe::Matrix camera_pose2;
  camera_pose2.resize(1, 7);
  camera_pose2.row(0) << 0, 0, 0, forward_x.q().x(), forward_x.q().y(),
      forward_x.q().z(), forward_x.q().w();
  GL_ASSERT_OK(graph.AddPacketToInputStream(
      "camera_pose",
      mediapipe::MakePacket<mediapipe::Matrix>(camera_pose2).At(t2)));
  GL_ASSERT_OK(graph.AddPacketToInputStream(
      "camera_params",
      mediapipe::MakePacket<mediapipe::Matrix>(camera_params).At(t2)));
  GL_ASSERT_OK(
      graph.AddPacketToInputStream("landmarks", input_landmarks_packet.At(t2)));
  GL_ASSERT_OK(
      graph.AddPacketToInputStream("image_frame", input_frame_packet.At(t2)));

  mediapipe::Packet output2;
  ASSERT_TRUE(poller.Next(&output2));
  ASSERT_TRUE(output2.At(t2).Get<bool>());

  mediapipe::Timestamp t3(3000);

  // Try another image which has no obstacles present.
  mediapipe::Matrix camera_pose3;
  camera_pose3.resize(1, 7);
  camera_pose3.row(0) << 0, 0, 0, forward_x.q().x(), forward_x.q().y(),
      forward_x.q().z(), forward_x.q().w();
  GL_ASSERT_OK(graph.AddPacketToInputStream(
      "camera_pose",
      mediapipe::MakePacket<mediapipe::Matrix>(camera_pose3).At(t3)));
  GL_ASSERT_OK(graph.AddPacketToInputStream(
      "camera_params",
      mediapipe::MakePacket<mediapipe::Matrix>(camera_params).At(t3)));
  GL_ASSERT_OK(
      graph.AddPacketToInputStream("landmarks", input_landmarks_packet.At(t3)));

  GL_ASSERT_OK_AND_ASSIGN(
      auto input_frame2,
      util::LoadImageFrame(GetTestDataPath("guideline1.png")));

  auto input_frame2_packet =
      mediapipe::MakePacket<mediapipe::ImageFrame>(std::move(input_frame2));
  GL_ASSERT_OK(
      graph.AddPacketToInputStream("image_frame", input_frame2_packet.At(t3)));

  mediapipe::Packet output3;
  ASSERT_TRUE(poller.Next(&output3));
  ASSERT_FALSE(output3.At(t3).Get<bool>());
}

}  // namespace
}  // namespace guideline::vision
