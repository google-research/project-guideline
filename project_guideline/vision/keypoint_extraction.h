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

#ifndef PROJECT_GUIDELINE_VISION_KEYPOINT_EXTRACTION_H_
#define PROJECT_GUIDELINE_VISION_KEYPOINT_EXTRACTION_H_

#include <vector>

#include <opencv2/core.hpp>  // keep include
#include "absl/status/statusor.h"
#include "Eigen/Core"
#include "project_guideline/proto/guideline_engine_config.pb.h"

namespace guideline::vision {

// Extracts guideline keypoints from the given segmentation mask.
// Returns the list of keypoints, where each Vector3f contains an [x, y]
// relative image frame coordinate between [0, 1], and the z value is a
// confidence between [0, 1].
absl::StatusOr<const std::vector<Eigen::Vector3f>> ExtractLineKeypointsFromMask(
    const cv::Mat_<float>& mask,
    const KeypointExtractorOptions& keypoint_extractor_options);

}  // namespace guideline::vision

#endif  // PROJECT_GUIDELINE_VISION_KEYPOINT_EXTRACTION_H_
