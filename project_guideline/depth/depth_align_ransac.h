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

#ifndef PROJECT_GUIDELINE_DEPTH_DEPTH_ALIGN_RANSAC_H_
#define PROJECT_GUIDELINE_DEPTH_DEPTH_ALIGN_RANSAC_H_

#include <vector>

#include "absl/status/statusor.h"
#include "Eigen/Core"
#include "project_guideline/camera/camera_model.h"
#include "project_guideline/motion/tracking_feature.h"
#include "project_guideline/util/image.h"

namespace guideline::depth {

struct DepthAlignParams {
  double scale = 1.0;
  double shift = 0.0;
};

absl::StatusOr<DepthAlignParams> DepthAlignRansac(
    const util::DepthImage& depth_map,
    const std::vector<motion::TrackingFeature>& tracking_features,
    const camera::CameraModel& camera_model);

absl::StatusOr<DepthAlignParams> DepthAlignRansacImageNormalized(
    const cv::Mat_<float>& depth_map, bool depth_map_is_inverse_depth,
    const std::vector<Eigen::Vector3f>& image_normalized_tracking_features);

}  // namespace guideline::depth

#endif  // PROJECT_GUIDELINE_DEPTH_DEPTH_ALIGN_RANSAC_H_
