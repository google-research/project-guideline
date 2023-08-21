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

#include "project_guideline/util/image.h"

#include <memory>

#include <opencv2/imgproc.hpp>
#include "absl/log/check.h"

namespace guideline::util {

namespace {
void CheckMatImageFormat(const cv::Mat& mat, ImageFormat format, uint16_t width,
                         uint16_t height) {
  CHECK_EQ(mat.cols, width);
  CHECK_EQ(mat.rows, height);
  switch (format) {
    case ImageFormat::kEmpty:
      CHECK(mat.empty());
      CHECK(mat.size().empty());
      break;
    case ImageFormat::kRGB:
      CHECK_EQ(mat.type(), CV_8UC3);
      break;
    case ImageFormat::kRGBA:
      CHECK_EQ(mat.type(), CV_8UC4);
      break;
    case ImageFormat::kUINT8:
      CHECK_EQ(mat.type(), CV_8UC1);
      break;
    case ImageFormat::kUINT16:
      CHECK_EQ(mat.type(), CV_16UC1);
      break;
    case ImageFormat::kFP32:
      CHECK_EQ(mat.type(), CV_32FC1);
      break;
  }
}
}  // namespace

Image::Image() : format_(ImageFormat::kEmpty), width_(0), height_(0), data_() {
  CheckMatImageFormat(*data_, format_, width_, height_);
}

Image::Image(ImageFormat format, uint16_t width, uint16_t height,
             std::unique_ptr<const cv::Mat> data, const ImageMetadata& metadata,
             std::unique_ptr<ScopedData> scoped_data)
    : format_(format),
      width_(width),
      height_(height),
      data_(std::move(data)),
      metadata_(metadata),
      scoped_data_(std::move(scoped_data)) {
  CheckMatImageFormat(*data_, format_, width_, height_);
}

Image RGBAToRGB(const Image& rgba) {
  CHECK(rgba.format() == ImageFormat::kRGBA) << "Image must be RGBA";
  std::unique_ptr<cv::Mat> rgb = std::make_unique<cv::Mat>();
  cv::cvtColor(rgba.data(), *rgb, cv::COLOR_RGBA2RGB);
  return util::Image(util::ImageFormat::kRGB, rgba.width(), rgba.height(),
                     std::move(rgb), rgba.metadata());
}

}  // namespace guideline::util
