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

#include <limits>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/calculator_runner.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/packet.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "project_guideline/testing/images.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/image_io.h"
#include "project_guideline/util/mediapipe.h"
#include "project_guideline/vision/models/guideline_tflite_embed.h"

namespace guideline::vision {
namespace {

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

TEST(ImageSegmentationCalculator, SegmentImage) {
  auto calculator_node =
      ParseTextProtoOrDie<CalculatorGraphConfig::Node>(absl::Substitute(
          R"pb(
            calculator: "ImageSegmentationCalculator"
            input_stream: "IMAGE:input"
            output_stream: "MASK:mask"
            node_options: {
              [type.googleapis.com/
               guideline.vision.ImageSegmentationCalculatorOptions]: {
                model_file: { file_path: "$0" }
                output_mask_index: 1
              }
            }
          )pb",
          GetModelPath("guideline.tflite")));
  mediapipe::CalculatorRunner runner(calculator_node);

  GL_ASSERT_OK_AND_ASSIGN(
      auto input_frame,
      util::LoadImageFrame(GetTestDataPath("guideline1.png")));
  auto input_frame_packet =
      mediapipe::MakePacket<mediapipe::ImageFrame>(std::move(input_frame));
  runner.MutableInputs()->Tag("IMAGE").packets.push_back(
      input_frame_packet.At(mediapipe::Timestamp(1)));

  GL_ASSERT_OK(runner.Run());

  const auto& outputs = runner.Outputs();
  EXPECT_EQ(outputs.NumEntries(), 1);
  const auto& output_mask =
      outputs.Tag("MASK").packets[0].Get<mediapipe::ImageFrame>();

  constexpr double kMaskImageScale = 65535.;
  GL_ASSERT_OK_AND_ASSIGN(
      auto mask_golden,
      util::LoadFloatImagePng(GetTestDataPath("guideline1_mask_golden.png"),
                              kMaskImageScale));

  util::Image mask_image = util::CopyImageFrameToImage(output_mask);
  EXPECT_TRUE(ImagesAlmostEqual(mask_golden, mask_image));
}

TEST(ImageSegmentationCalculator, ModelPointer) {
  auto calculator_node =
      ParseTextProtoOrDie<CalculatorGraphConfig::Node>(absl::Substitute(
          R"pb(
            calculator: "ImageSegmentationCalculator"
            input_stream: "IMAGE:input"
            output_stream: "MASK:mask"
            node_options: {
              [type.googleapis.com/
               guideline.vision.ImageSegmentationCalculatorOptions]: {
                model_file: { content_pointer { pointer: $0 length: $1 } }
                output_mask_index: 1
              }
            }
          )pb",
          reinterpret_cast<size_t>(guideline_tflite_embed_create()[0].data),
          guideline_tflite_embed_create()[0].size));
  mediapipe::CalculatorRunner runner(calculator_node);

  GL_ASSERT_OK_AND_ASSIGN(
      auto input_frame,
      util::LoadImageFrame(GetTestDataPath("guideline1.png")));
  auto input_frame_packet =
      mediapipe::MakePacket<mediapipe::ImageFrame>(std::move(input_frame));
  runner.MutableInputs()->Tag("IMAGE").packets.push_back(
      input_frame_packet.At(mediapipe::Timestamp(1)));

  GL_ASSERT_OK(runner.Run());
}

TEST(ImageSegmentationCalculator, Depth) {
  auto calculator_node =
      ParseTextProtoOrDie<CalculatorGraphConfig::Node>(absl::Substitute(
          R"pb(
            calculator: "ImageSegmentationCalculator"
            input_stream: "IMAGE:input"
            output_stream: "MASK:mask"
            node_options: {
              [type.googleapis.com/
               guideline.vision.ImageSegmentationCalculatorOptions]: {
                model_file: { file_path: "$0" }
              }
            }
          )pb",
          GetModelPath("depth.tflite")));
  mediapipe::CalculatorRunner runner(calculator_node);

  GL_ASSERT_OK_AND_ASSIGN(
      auto input_frame,
      util::LoadImageFrame(GetTestDataPath("obstacle_synthetic.png")));
  auto input_frame_packet =
      mediapipe::MakePacket<mediapipe::ImageFrame>(std::move(input_frame));
  runner.MutableInputs()->Tag("IMAGE").packets.push_back(
      input_frame_packet.At(mediapipe::Timestamp(1)));

  GL_ASSERT_OK(runner.Run());

  const auto& outputs = runner.Outputs();
  EXPECT_EQ(outputs.NumEntries(), 1);
  const auto& output_mask =
      outputs.Tag("MASK").packets[0].Get<mediapipe::ImageFrame>();

  constexpr double kDepthImageScale = 100000.;
  GL_ASSERT_OK_AND_ASSIGN(
      auto depth_golden,
      util::LoadFloatImagePng(
          GetTestDataPath("obstacle_synthetic_depth_inverse_golden.png"),
          kDepthImageScale));

  // Clamp the image since some range is lost when saving the golden.
  util::Image depth_image =
      ClampImage(util::CopyImageFrameToImage(output_mask), 0,
                 std::numeric_limits<uint16_t>::max() / kDepthImageScale);

  // Uncomment to update golden.
  // GL_ASSERT_OK(util::SaveFloatImagePng(depth_image,
  //     "/tmp/obstacle_synthetic_depth_inverse_golden.png",
  //      kDepthImageScale, 0.0));

  EXPECT_TRUE(ImagesAlmostEqual(depth_golden, depth_image, 1e-5));
}

}  // namespace
}  // namespace guideline::vision
