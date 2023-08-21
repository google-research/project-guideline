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

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "Eigen/Core"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/formats/matrix.h"
#include "project_guideline/camera/cv_camera_model.h"
#include "project_guideline/depth/point_cloud_util.h"
#include "project_guideline/environment/obstacle_utils.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/transformation.h"
#include "project_guideline/vision/calculators/occupancy_map_calculator_options.pb.h"

namespace guideline::vision {

namespace {
using Eigen::Vector2d;
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
constexpr char kOccupancyMapTag[] = "OCCUPANCY_MAP";
constexpr char kOccupiedTag[] = "OCCUPIED";
}  // namespace

class OccupancyMapCalculator : public CalculatorBase {
 public:
  OccupancyMapCalculator() = default;
  ~OccupancyMapCalculator() override = default;
  static absl::Status GetContract(CalculatorContract* cc);

  absl::Status Open(CalculatorContext* cc) override;
  absl::Status Process(CalculatorContext* cc) override;
};
REGISTER_CALCULATOR(OccupancyMapCalculator);

absl::Status OccupancyMapCalculator::GetContract(CalculatorContract* cc) {
  RET_CHECK(cc->Inputs().HasTag(kDepthImageTag));
  RET_CHECK(cc->Inputs().HasTag(kCameraPoseTag));
  RET_CHECK(cc->Inputs().HasTag(kCameraParamsTag));

  cc->Inputs().Tag(kDepthImageTag).Set<ImageFrame>();
  cc->Inputs().Tag(kCameraPoseTag).Set<Matrix>();
  cc->Inputs().Tag(kCameraParamsTag).Set<Matrix>();
  cc->Outputs().Tag(kOccupancyMapTag).Set<Matrix>();
  cc->Outputs().Tag(kOccupiedTag).Set<bool>();
  return absl::OkStatus();
}

absl::Status OccupancyMapCalculator::Open(CalculatorContext* cc) {
  return absl::OkStatus();
}

absl::Status OccupancyMapCalculator::Process(CalculatorContext* cc) {
  if (cc->Inputs().Tag(kDepthImageTag).IsEmpty()) {
    return absl::OkStatus();
  }

  const auto& options = cc->Options<OccupancyMapCalculatorOptions>();

  const auto& input_frame = cc->Inputs().Tag(kDepthImageTag).Get<ImageFrame>();
  RET_CHECK_EQ(input_frame.Format(), ImageFormat::VEC32F1)
      << "Input image must be float32 depth image";
  cv::Mat input_mat = MatView(&input_frame);
  util::DepthImage depth_image(input_frame.Width(), input_frame.Height(),
                               std::make_unique<cv::Mat>(input_mat));

  const auto& camera_pose_vector =
      cc->Inputs().Tag(kCameraPoseTag).Get<Matrix>().cast<double>().row(0);
  RET_CHECK_EQ(camera_pose_vector.cols(), 7)
      << "Camera pose must have 7 values [x, y, z, qx, qy, qz, qw]";

  const auto& camera_params_vector =
      cc->Inputs().Tag(kCameraParamsTag).Get<Matrix>().cast<double>().row(0);
  RET_CHECK_EQ(camera_params_vector.cols(), 6)
      << "Camera params must have 6 values [w, h, fx, fy, cx, cy]";

  Transformation world_t_camera = Transformation(
      util::Quaternion(camera_pose_vector(6), camera_pose_vector(3),
                       camera_pose_vector(4), camera_pose_vector(5)),
      camera_pose_vector.head<3>());

  camera::CvCameraModel camera_model(camera_params_vector(0),
                                     camera_params_vector(1),
                                     camera_params_vector.segment<4>(2));

  auto point_cloud =
      depth::DepthMapToPointCloud(depth_image, world_t_camera, camera_model);

  if (!point_cloud.ok()) {
    return absl::OkStatus();
  }

  absl::flat_hash_map<std::pair<float, float>, int> occupancy_grids =
      environment::GetClearanceZone(options.width_meters(),
                                    options.depth_meters(), world_t_camera);

  float clearance_zone_top = world_t_camera.p().z() + options.top_meters();
  float clearance_zone_bottom =
      world_t_camera.p().z() + options.bottom_meters();

  // Compute the occupancy map within the clearance zone, based on
  // the 3D point cloud.
  for (const auto& point : *point_cloud) {
    if (point.confidence <= options.point_confidence_threshold()) {
      continue;
    }
    if (point.coordinate.z() < clearance_zone_bottom ||
        point.coordinate.z() > clearance_zone_top) {
      continue;
    }
    const std::pair<float, float> closest_grid(round(point.coordinate.x()),
                                               round(point.coordinate.y()));
    if (occupancy_grids.contains(closest_grid)) {
      ++occupancy_grids[closest_grid];
    }
  }

  Matrix matrix;
  std::vector<float> occupancy_map;
  occupancy_map.reserve(occupancy_grids.size() * 3);
  bool occupied = false;
  for (auto it = occupancy_grids.begin(); it != occupancy_grids.end(); ++it) {
    int occupancy = it->second;
    if (occupancy > options.occupancy_threshold()) {
      occupied = true;
    }
    occupancy_map.push_back(it->first.first);
    occupancy_map.push_back(it->first.second);
    occupancy_map.push_back(occupancy);
  }

  Matrix occupancy_map_matrix =
      Eigen::Map<Matrix>(occupancy_map.data(), 1, occupancy_map.size());

  cc->Outputs()
      .Tag(kOccupancyMapTag)
      .AddPacket(mediapipe::MakePacket<Matrix>(occupancy_map_matrix)
                     .At(cc->InputTimestamp()));

  cc->Outputs()
      .Tag(kOccupiedTag)
      .AddPacket(
          mediapipe::MakePacket<bool>(occupied).At(cc->InputTimestamp()));

  return absl::OkStatus();
}

}  // namespace guideline::vision
