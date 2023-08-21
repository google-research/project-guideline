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

#include "project_guideline/util/image_yuv_conversion.h"

#include <opencv2/core.hpp>  // keep include
#include "absl/log/check.h"
#include "libyuv/convert.h"  // keep include

namespace guideline::util {

absl::StatusOr<Image> ConvertYuvToRgb(const int image_width,
                                      const int image_height,
                                      const std::vector<ImagePlane>& planes,
                                      uint64_t image_timestamp_ns) {
  CHECK_EQ(planes.size(), 3) << "YUV image must have 3 planes";

  if (image_width % 2 != 0 || image_height % 2 != 0) {
    return absl::InvalidArgumentError("Image must have even dimensions");
  }

  bool is_yuv420_interleaved =
      // Same row stride for all planes
      planes[0].row_stride == planes[1].row_stride &&
      planes[0].row_stride == planes[2].row_stride &&
      // Pixels packed with U and V interleaved.
      planes[0].pixel_stride == 1 && planes[1].pixel_stride == 2 &&
      planes[2].pixel_stride == 2 &&
      std::abs(planes[1].data - planes[2].data) == 1;

  bool is_yuv420_noninterleaved =
      // U and V have 1/2 stride of Y.
      planes[0].row_stride == 2 * planes[1].row_stride &&
      planes[0].row_stride == 2 * planes[2].row_stride &&
      // Pixels packed, not interleaved.
      planes[0].pixel_stride == 1 && planes[1].pixel_stride == 1 &&
      planes[2].pixel_stride == 1 &&
      // No padding after Y.
      planes[0].data_length == image_height * planes[0].row_stride &&
      // U and V are 1/4 size of Y.
      planes[0].data_length == 4 * planes[1].data_length &&
      planes[0].data_length == 4 * planes[2].data_length;

  bool is_split_nv21 =
      is_yuv420_interleaved && planes[2].data + 1 == planes[1].data;

  bool is_split_nv12 =
      is_yuv420_interleaved && planes[1].data + 1 == planes[2].data;

  auto mat = std::make_unique<cv::Mat>(image_height, image_width, CV_8UC3);

  int result = 0;
  if (is_split_nv21) {
    result = libyuv::NV21ToRAW(planes[0].data, planes[0].row_stride,
                               planes[2].data, planes[2].row_stride, mat->data,
                               image_width * mat->channels(), image_width,
                               image_height);
  } else if (is_split_nv12) {
    result = libyuv::NV12ToRAW(planes[0].data, planes[0].row_stride,
                               planes[1].data, planes[1].row_stride, mat->data,
                               image_width * mat->channels(), image_width,
                               image_height);
  } else if (is_yuv420_interleaved || is_yuv420_noninterleaved) {
    result = libyuv::I420ToRAW(
        planes[0].data, planes[0].row_stride, planes[1].data,
        planes[1].row_stride, planes[2].data, planes[2].row_stride, mat->data,
        image_width * mat->channels(), image_width, image_height);
  } else {
    return absl::InvalidArgumentError(
        "Input image must be NV21, NV12, or I420");
  }

  if (result != 0) {
    return absl::InternalError("Failed to convert image from YUV");
  }

  ImageMetadata metadata;
  metadata.timestamp_ns = image_timestamp_ns;

  return Image(ImageFormat::kRGB, image_width, image_height, std::move(mat),
               metadata);
}

}  // namespace guideline::util
