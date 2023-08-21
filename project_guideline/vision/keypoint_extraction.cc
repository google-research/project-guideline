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

#include "project_guideline/vision/keypoint_extraction.h"

#include <algorithm>

namespace guideline::vision {

namespace {
struct Moments {
  float centroid_x = 0.;
  float centroid_y = 0.;
  float sum = 0.;
};

// Calculates the first order image moments for the given mask, between the
// given start and end rows.
Moments CalculateMoments(const cv::Mat_<float>& mask, int start_row_index,
                         int end_row_index) {
  Moments moments;

  constexpr float kConfidenceThreshold = 0.5f;
  for (int y = start_row_index; y < end_row_index; ++y) {
    for (int x = 0; x < mask.cols; ++x) {
      float value = mask(y, x);
      if (value < kConfidenceThreshold) {
        value = 0;
      }

      moments.sum += value;
      moments.centroid_x += x * value;
      moments.centroid_y += y * value;
    }
  }

  if (moments.sum > 0) {
    moments.centroid_x /= moments.sum;
    moments.centroid_y /= moments.sum;
  } else {
    moments.centroid_x = 0;
    moments.centroid_y = 0;
  }

  return moments;
}

std::vector<Eigen::Vector3f> ExtractHorizontalBlockBasedKeypoint(
    const cv::Mat_<float>& mask, int block_size, int block_overlap) {
  std::vector<Eigen::Vector3f> keypoints;

  int num_blocks = (int)(1.0f * mask.rows / (block_size - block_overlap));
  std::vector<int> row_indices(num_blocks);
  int start = -block_overlap;
  int end = mask.rows - block_overlap;
  float increment = 1.0f * (end - start) / (num_blocks - 1);
  for (int i = 0; i < num_blocks; i++) {
    row_indices[i] = round(start + i * increment);
  }

  // TODO(b/211627343): Improve the keypoint logic.
  // The constants are based on casual observations that a point with sum
  // below kMinSumThreshold tends to be a spurious point not on line, and
  // above kHighConfidenceSum is almost always part of the line.
  const float kMinSumThreshold = 2.0f;
  const float kHighConfidenceSum = 10.0f;

  for (int i = 0; i < row_indices.size(); i++) {
    int row_index = row_indices[i];
    int end_row_index = std::min(row_index + block_size, mask.rows);
    int start_row_index = std::max(row_index, 0);
    auto moments = CalculateMoments(mask, start_row_index, end_row_index);
    if (moments.sum > kMinSumThreshold) {
      // Add 0.5 to the centroid indices so the center position of the pixels
      // are used for keypoint placement.
      keypoints.emplace_back((moments.centroid_x + 0.5) / mask.cols,
                             (moments.centroid_y + 0.5) / mask.rows,
                             std::min(1.0f, moments.sum / kHighConfidenceSum));
    }
  }

  return keypoints;
}
}  // namespace

absl::StatusOr<const std::vector<Eigen::Vector3f>> ExtractLineKeypointsFromMask(
    const cv::Mat_<float>& mask,
    const KeypointExtractorOptions& keypoint_extractor_options) {
  switch (keypoint_extractor_options.keypoint_extractor_options_case()) {
    case KeypointExtractorOptions::KEYPOINT_EXTRACTOR_OPTIONS_NOT_SET:
      // fall-through
    case KeypointExtractorOptions::kHorizontalBlockKeypointExtractorOptions: {
      const HorizontalBlockKeypointExtractorOptions&
          horizontal_block_keypoint_extractor_options =
              keypoint_extractor_options
                  .horizontal_block_keypoint_extractor_options();
      return ExtractHorizontalBlockBasedKeypoint(
          mask, horizontal_block_keypoint_extractor_options.block_size(),
          horizontal_block_keypoint_extractor_options.block_overlap());
    }
  }
  return absl::InvalidArgumentError("Invalid keypoint extractor option.");
}

}  // namespace guideline::vision
