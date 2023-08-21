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

#include "project_guideline/testing/images.h"

#include "absl/log/check.h"

namespace guideline {

bool ImagesEqual(const util::Image& image1, const util::Image& image2) {
  CHECK(image1.format() == image2.format()) << "Images must have same format";
  CHECK(image1.width() == image2.width() && image1.height() == image2.height())
      << "Images must have same dimensions";

  cv::Mat diff = cv::abs(image1.data() - image2.data());
  cv::Scalar diff_sum_channels = cv::sum(diff);
  cv::Scalar total_diff_sum;
  cv::reduce(diff_sum_channels, total_diff_sum, 1, cv::REDUCE_SUM);

  return total_diff_sum[0] < std::numeric_limits<double>::epsilon();
}

bool ImagesAlmostEqual(const util::Image& image1, const util::Image& image2,
                       double tolerance) {
  CHECK(image1.format() == image2.format()) << "Images must have same format";
  CHECK(image1.width() == image2.width() && image1.height() == image2.height())
      << "Images must have same dimensions";

  cv::Mat diff = image1.data() - image2.data();
  cv::pow(diff, 2, diff);

  cv::Scalar squared_diff_sum_channels = cv::sum(diff);
  cv::Scalar total_squared_diff_sum;
  cv::reduce(squared_diff_sum_channels, total_squared_diff_sum, 1,
             cv::REDUCE_SUM);

  double avg_squared_diff = total_squared_diff_sum[0] / diff.total();
  return avg_squared_diff < tolerance;
}

util::Image ClampImage(const util::Image& image, float min, float max) {
  auto mat = std::make_unique<cv::Mat>(
      cv::min(cv::max(image.data().clone(), min), max));
  return util::Image(image.format(), image.width(), image.height(),
                     std::move(mat), image.metadata());
}

}  // namespace guideline
