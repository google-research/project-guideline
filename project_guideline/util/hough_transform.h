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

// Specialized implementation of the Hough Transform for guideline detection
// from the segmentation model mask output.

#ifndef PROJECT_GUIDELINE_UTIL_HOUGH_TRANSFORM_H_
#define PROJECT_GUIDELINE_UTIL_HOUGH_TRANSFORM_H_

#include <cmath>
#include <cstdint>
#include <vector>

#include <opencv2/core/mat.hpp>
#include "Eigen/Core"  // keep include

namespace guideline::util {

// Result of a Hough Transform.
struct HoughTransformResult {
  Eigen::Vector2f bottom_intercept;
  Eigen::Vector2f top_intercept;
  float score;

  // Note the intercept coordinates are relative with coordinates in range
  // [0, 1]. Bottom right of the mask is (1, 1).
  HoughTransformResult(const Eigen::Vector2f& bottom_intercept,
                       const Eigen::Vector2f& top_intercept, float score)
      : bottom_intercept(bottom_intercept),
        top_intercept(top_intercept),
        score(score) {}

  float GetAngleRadians(int image_width, int image_height) const {
    auto diff = top_intercept - bottom_intercept;
    // Compute the angle, with 0 degrees pointing up and -90 degrees pointing to
    // the left.
    return std::atan2(diff.x() * image_width, -diff.y() * image_height);
  }

  float GetAngleDegrees(int image_width, int image_height) const {
    return GetAngleRadians(image_width, image_height) * 180.0f / M_PI;
  }
};

class HoughTransform {
 public:
  HoughTransform(uint16_t position_steps, uint16_t angle_steps,
                 float confidence_threshold, uint16_t mask_width,
                 uint16_t mask_height, uint16_t num_segments,
                 float segment_overlap);

  std::vector<HoughTransformResult> Process(const cv::Mat& mask);

 private:
  float CachedCos(int t);
  float CachedSin(int t);

  const uint16_t position_steps_;
  const uint16_t angle_steps_;
  const float confidence_threshold_;
  const uint16_t mask_width_;
  const uint16_t mask_height_;
  const uint16_t bottom_quadrant_;
  const uint16_t top_quadrant_;
  const uint16_t segment_height_;
  const uint16_t segment_overlap_;
  const uint16_t num_segments_;
  const uint16_t hough_width_;
  const uint16_t half_hough_width_;
  const Eigen::Vector2f center_;

  std::vector<double> sin_cache_;
  std::vector<double> cos_cache_;
  Eigen::MatrixXf hough_transform_;
};

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_HOUGH_TRANSFORM_H_
