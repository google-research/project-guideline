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
#include <memory>
#include <tuple>
#include <utility>

#include <opencv2/core.hpp>  // keep include
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "Eigen/Core"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/formats/matrix.h"
#include "mediapipe/framework/packet.h"
#include "mediapipe/framework/port/ret_check.h"
#include "project_guideline/camera/cv_camera_model.h"
#include "project_guideline/depth/depth_align_ransac.h"
#include "project_guideline/util/transformation.h"
#include "project_guideline/vision/calculators/depth_alignment_calculator_options.pb.h"

namespace guideline::vision {

namespace {
using depth::DepthAlignParams;
using depth::DepthAlignRansacImageNormalized;
using ::drishti::LandmarkList;
using ::mediapipe::CalculatorBase;
using ::mediapipe::CalculatorContext;
using ::mediapipe::CalculatorContract;
using ::mediapipe::ImageFormat;
using ::mediapipe::ImageFrame;
using ::mediapipe::Matrix;
using ::mediapipe::formats::MatView;
using util::Transformation;

constexpr char kDepthImageTag[] = "DEPTH_IMAGE";
constexpr char kCameraPoseTag[] = "CAMERA_POSE";
constexpr char kCameraParamsTag[] = "CAMERA_PARAMS";
constexpr char kLandmarksTag[] = "LANDMARKS";
constexpr char kFallbackInvScaleShiftTag[] = "FALLBACK_INV_SCALE_SHIFT";
constexpr char kInverseScaleShiftTag[] = "INV_SCALE_SHIFT";
constexpr char kDepthImageAlignedTag[] = "DEPTH_IMAGE_ALIGNED";
constexpr char kDepthAlignmentSuccessTag[] = "DEPTH_ALIGNMENT_SUCCESS";
}  // namespace

class DepthAlignmentCalculator : public CalculatorBase {
 public:
  DepthAlignmentCalculator() = default;
  ~DepthAlignmentCalculator() override = default;
  static absl::Status GetContract(CalculatorContract* cc);

  absl::Status Open(CalculatorContext* cc) override;
  absl::Status Process(CalculatorContext* cc) override;
};
REGISTER_CALCULATOR(DepthAlignmentCalculator);

absl::Status DepthAlignmentCalculator::GetContract(CalculatorContract* cc) {
  RET_CHECK(cc->Inputs().HasTag(kDepthImageTag));
  RET_CHECK(cc->Inputs().HasTag(kLandmarksTag));
  RET_CHECK(cc->Inputs().HasTag(kCameraPoseTag));
  RET_CHECK(cc->Inputs().HasTag(kCameraParamsTag));

  cc->Inputs().Tag(kLandmarksTag).Set<LandmarkList>();
  cc->Inputs().Tag(kDepthImageTag).Set<ImageFrame>();
  cc->Inputs().Tag(kCameraPoseTag).Set<Matrix>();
  cc->Inputs().Tag(kCameraParamsTag).Set<Matrix>();
  cc->Inputs()
      .Tag(kFallbackInvScaleShiftTag)
      .Set<std::pair<float, float>>()
      .Optional();
  cc->Outputs().Tag(kInverseScaleShiftTag).Set<std::pair<float, float>>();
  cc->Outputs().Tag(kDepthImageAlignedTag).Set<ImageFrame>();
  cc->Outputs().Tag(kDepthAlignmentSuccessTag).Set<bool>().Optional();
  return absl::OkStatus();
}

absl::Status DepthAlignmentCalculator::Open(CalculatorContext* cc) {
  return absl::OkStatus();
}

absl::Status DepthAlignmentCalculator::Process(CalculatorContext* cc) {
  if (cc->Inputs().Tag(kDepthImageTag).IsEmpty()) {
    return absl::OkStatus();
  }

  const auto& options = cc->Options<DepthAlignmentCalculatorOptions>();

  const auto& input_frame = cc->Inputs().Tag(kDepthImageTag).Get<ImageFrame>();

  RET_CHECK_EQ(input_frame.Format(), ImageFormat::VEC32F1)
      << "Input image must be float32 depth image";
  cv::Mat input_mat = MatView(&input_frame);

  const auto& camera_pose_vector =
      cc->Inputs().Tag(kCameraPoseTag).Get<Matrix>().cast<double>().row(0);
  RET_CHECK_EQ(camera_pose_vector.cols(), 7)
      << "Camera pose must have 7 values [x, y, z, qx, qy, qz, qw]";

  const auto& camera_params_vector =
      cc->Inputs().Tag(kCameraParamsTag).Get<Matrix>().cast<double>().row(0);
  RET_CHECK_EQ(camera_params_vector.cols(), 6)
      << "Camera params must have 6 values [w, h, fx, fy, cx, cy]";

  Transformation world_t_camera = Transformation(
      util::Quaternion(camera_pose_vector(3), camera_pose_vector(0),
                       camera_pose_vector(1), camera_pose_vector(2)),
      camera_pose_vector.head<3>());

  camera::CvCameraModel camera_model(camera_params_vector(0),
                                     camera_params_vector(1),
                                     camera_params_vector.segment<4>(2));

  RET_CHECK_EQ(input_frame.Width(), camera_model.image_width())
      << "Image height does not match camera model";
  RET_CHECK_EQ(input_frame.Height(), camera_model.image_height())
      << "Image width does not match camera model";

  std::vector<Eigen::Vector3f> image_normalized_features;
  const auto& landmarks = cc->Inputs().Tag(kLandmarksTag).Get<LandmarkList>();
  for (const auto& landmark : landmarks.landmark()) {
    Transformation world_t_landmark({landmark.x(), landmark.y(), landmark.z()});
    Transformation camera_t_landmark =
        world_t_camera.Inverse() * world_t_landmark;

    Eigen::Vector2d pixel;
    if (camera_model.PointToPixel(camera_t_landmark.p(), pixel)) {
      image_normalized_features.emplace_back(pixel.x(), pixel.y(),
                                             landmark.z());
    }
  }

  float scale = 1.0;
  float shift = 0.0;
  absl::StatusOr<DepthAlignParams> params = DepthAlignRansacImageNormalized(
      input_mat, options.input_inverse_depth(), image_normalized_features);
  if (!params.ok()) {
    // If the alignment fails, use the fallback scale/shift params if they have
    // been provided.
    if (cc->Inputs().HasTag(kFallbackInvScaleShiftTag) &&
        !cc->Inputs().Tag(kFallbackInvScaleShiftTag).IsEmpty()) {
      std::tie(scale, shift) = cc->Inputs()
                                   .Tag(kFallbackInvScaleShiftTag)
                                   .Get<std::pair<float, float>>();
    } else {
      return params.status();
    }
  } else {
    scale = params->scale;
    shift = params->shift;
  }

  if (cc->Outputs().HasTag(kDepthAlignmentSuccessTag)) {
    cc->Outputs()
        .Tag(kDepthAlignmentSuccessTag)
        .AddPacket(
            mediapipe::MakePacket<bool>(params.ok()).At(cc->InputTimestamp()));
  }

  cc->Outputs()
      .Tag(kInverseScaleShiftTag)
      .AddPacket(mediapipe::MakePacket<std::pair<float, float>>(scale, shift)
                     .At(cc->InputTimestamp()));

  auto output_frame = std::make_unique<ImageFrame>(
      input_frame.Format(), input_mat.cols, input_mat.rows);
  cv::Mat output_mat = MatView(output_frame.get());

  auto output_iter = output_mat.begin<float>();
  for (auto input_iter = input_mat.begin<float>();
       input_iter != input_mat.end<float>(); ++input_iter, ++output_iter) {
    float depth = *input_iter;
    float inv_depth;
    if (options.input_inverse_depth()) {
      inv_depth = depth;
    } else {
      inv_depth = depth < std::numeric_limits<float>::epsilon()
                      ? std::numeric_limits<float>::max()
                      : 1.0 / depth;
    }
    const float pred_inv_depth = inv_depth * scale + shift;
    if (options.output_inverse_depth()) {
      *output_iter = pred_inv_depth;
    } else {
      const float pred_depth =
          pred_inv_depth < std::numeric_limits<float>::epsilon()
              ? std::numeric_limits<float>::max()
              : 1.0 / pred_inv_depth;
      *output_iter = pred_depth;
    }
  }

  cc->Outputs()
      .Tag(kDepthImageAlignedTag)
      .Add(output_frame.release(), cc->InputTimestamp());
  return absl::OkStatus();
}

}  // namespace guideline::vision
