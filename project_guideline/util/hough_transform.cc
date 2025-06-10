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

#include "project_guideline/util/hough_transform.h"

#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

#include <opencv2/core/mat.hpp>
#include "Eigen/Core"  // keep include

namespace guideline::util {

HoughTransform::HoughTransform(uint16_t position_steps, uint16_t angle_steps,
                               float confidence_threshold, uint16_t mask_width,
                               uint16_t mask_height, uint16_t num_segments,
                               float segment_overlap_pct)
    : position_steps_(position_steps),
      angle_steps_(angle_steps),
      confidence_threshold_(confidence_threshold),
      mask_width_(mask_width),
      mask_height_(mask_height),
      bottom_quadrant_(angle_steps / 4),
      top_quadrant_(angle_steps * 3 / 4),
      segment_height_(
          mask_height *
          ((1.0f + (num_segments - 1) * segment_overlap_pct) / num_segments)),
      segment_overlap_(std::ceil(segment_overlap_pct * mask_height)),
      num_segments_(num_segments),
      hough_width_(std::hypot(mask_width, mask_height) * 2),
      half_hough_width_(std::hypot(mask_width, mask_height)),
      center_(mask_width / 2.0f, mask_height / 2.0f) {
  sin_cache_.resize(angle_steps_);
  cos_cache_.resize(angle_steps_);
  hough_transform_.resize(hough_width_, angle_steps_);

  for (int i = 0; i < angle_steps_; ++i) {
    float theta = i * std::numbers::pi / angle_steps_;
    sin_cache_[i] = std::sin(theta);
    cos_cache_[i] = std::cos(theta);
  }
}

float HoughTransform::CachedCos(int t) { return cos_cache_[std::abs(t)]; }

float HoughTransform::CachedSin(int t) {
  return std::copysign(sin_cache_[std::abs(t)], t);
}

std::vector<HoughTransformResult> HoughTransform::Process(const cv::Mat& mask) {
  std::vector<HoughTransformResult> results;
  results.reserve(num_segments_);

  for (int segment = 0; segment < num_segments_; ++segment) {
    hough_transform_.setZero();

    int segment_offset = segment * (segment_height_ - segment_overlap_);

    for (int position_x = 0; position_x < position_steps_; ++position_x) {
      int x = position_x * mask_width_ / position_steps_;
      for (int position_y = 0; position_y < position_steps_; ++position_y) {
        int y = position_y * segment_height_ / position_steps_ + segment_offset;
        float value = mask.at<float>(y, x);
        if (value >= confidence_threshold_) {
          for (int a = 0; a < angle_steps_; ++a) {
            float radius = ((x - center_.x()) * cos_cache_[a]) +
                           ((y - center_.y()) * sin_cache_[a]) +
                           half_hough_width_;
            int r1 = radius;
            if (r1 >= 0 && r1 < hough_width_ - 1) {
              float p = radius - r1;
              int r2 = r1 + 1;
              hough_transform_(r1, a) += (1 - p) * value * value;
              hough_transform_(r2, a) += p * value * value;
            }
          }
        }
      }
    }

    float best_value = 0;
    int best_a = 0;
    for (int r = 0; r < hough_width_; ++r) {
      for (int a = 0; a < angle_steps_; ++a) {
        float value = hough_transform_(r, a);
        if (value > best_value) {
          best_value = value;
          best_a = a;
        }
      }
    }

    bool connect_quadrants =
        best_a < bottom_quadrant_ || best_a > top_quadrant_;
    float value_threshold = 0.9 * best_value;
    int sum_r = 0;
    float sum_x = 0;
    float sum_y = 0;
    int point_count = 0;
    for (int r = 0; r < hough_width_; ++r) {
      for (int a = 0; a < angle_steps_; ++a) {
        float value = hough_transform_(r, a);
        if (value >= value_threshold) {
          if (connect_quadrants && a > top_quadrant_) {
            sum_r += hough_width_ - r;
            sum_x += CachedCos(a - angle_steps_);
            sum_y += CachedSin(a - angle_steps_);
          } else {
            sum_r += r;
            sum_x += CachedCos(a);
            sum_y += CachedSin(a);
          }
          ++point_count;
        }
      }
    }

    int best_r = sum_r / point_count;
    float theta = std::atan2(sum_y, sum_x);
    if (theta < 0) {
      theta += std::numbers::pi;
    }
    // Convert the line into top/bottom intercept.
    int r = best_r - half_hough_width_;
    if (connect_quadrants && theta > std::numbers::pi * 3 / 4) {
      r = -r;
    }
    float zero_intercept = r / std::cos(theta);
    float dx = std::tan(theta);
    float top_y = segment * (segment_height_ - segment_overlap_);
    float top_dy = top_y - center_.y();
    float bottom_dy = top_dy + segment_height_;

    float bottom_intercept = zero_intercept - bottom_dy * dx + center_.x();
    float top_intercept = zero_intercept - top_dy * dx + center_.x();

    results.emplace_back(
        Eigen::Vector2f(bottom_intercept / mask_width_,
                        (top_y + segment_height_) / mask_height_),
        Eigen::Vector2f(top_intercept / mask_width_, top_y / mask_height_),
        best_value);
  }

  return results;
}

}  // namespace guideline::util
