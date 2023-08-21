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

#include "project_guideline/util/image_io.h"

#include <opencv2/core.hpp>  // keep include
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "absl/status/status.h"
#include "absl/strings/str_format.h"

namespace guideline::util {

namespace {
void ApplyRotation(const cv::Mat& input, cv::Mat& output,
                   ImageRotation rotation) {
  int rotate_code;
  switch (rotation) {
    case ImageRotation::k90:
      rotate_code = cv::ROTATE_90_CLOCKWISE;
      break;
    case ImageRotation::k180:
      rotate_code = cv::ROTATE_180;
      break;
    case ImageRotation::k270:
      rotate_code = cv::ROTATE_90_COUNTERCLOCKWISE;
      break;
    case ImageRotation::kNone:
      output = input;
      return;
  }
  cv::rotate(input, output, rotate_code);
}
}  // namespace

absl::StatusOr<Image> LoadImage(const std::string& file_path, bool keep_alpha,
                                ImageRotation rotation) {
  const cv::Mat mat = cv::imread(
      file_path, keep_alpha ? cv::IMREAD_UNCHANGED : cv::IMREAD_COLOR);
  if (mat.data == nullptr) {
    return absl::InternalError(
        absl::StrFormat("Failed to read file at: %s", file_path));
  }
  ImageFormat format;
  switch (mat.type()) {
    case CV_8UC3:
      format = ImageFormat::kRGB;
      cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
      break;
    case CV_8UC4:
      format = ImageFormat::kRGBA;
      cv::cvtColor(mat, mat, cv::COLOR_BGRA2RGBA);
      break;
    case CV_8UC1:
      format = ImageFormat::kUINT8;
      break;
    case CV_16UC1:
      format = ImageFormat::kUINT16;
      break;
    case CV_32FC1:
      format = ImageFormat::kFP32;
      break;
    default:
      return absl::UnimplementedError(
          absl::StrFormat("CV image format not supported: %d", mat.type()));
  }
  auto rotated = std::make_unique<cv::Mat>();
  ApplyRotation(mat, *rotated, rotation);
  return Image(format, mat.size().width, mat.size().height, std::move(rotated));
}

absl::Status SaveImage(const Image& image, const std::string& file_path,
                       ImageRotation rotation) {
  cv::Mat mat;
  ApplyRotation(image.data(), mat, rotation);

  bool result;
  switch (image.format()) {
    case ImageFormat::kRGB: {
      cv::Mat bgr;
      cv::cvtColor(mat, bgr, cv::COLOR_RGB2BGR);
      result = cv::imwrite(file_path, bgr);
      break;
    }
    case ImageFormat::kRGBA: {
      cv::Mat bgra;
      cv::cvtColor(mat, bgra, cv::COLOR_RGBA2BGRA);
      result = cv::imwrite(file_path, bgra);
      break;
    }
    default: {
      result = cv::imwrite(file_path, mat);
    }
  }
  return result ? absl::OkStatus()
                : absl::InternalError(
                      absl::StrFormat("Failed to save image to %s", file_path));
}

absl::StatusOr<TypedImage<ImageFormat::kFP32, float>> LoadFloatImagePng(
    const std::string& png_file_path, double scale, double shift,
    ImageRotation rotation) {
  const cv::Mat mat = cv::imread(png_file_path, cv::IMREAD_ANYDEPTH);
  if (mat.data == nullptr) {
    return absl::InternalError(
        absl::StrFormat("Failed to read file at: %s", png_file_path));
  }
  if (mat.type() != CV_16UC1) {
    return absl::InvalidArgumentError("File is not 16-bit grayscale png");
  }

  cv::Mat fmat;
  mat.convertTo(fmat, CV_32FC1, 1.0 / scale, -shift);
  ApplyRotation(fmat, fmat, rotation);

  return DepthImage(mat.size().width, mat.size().height,
                    std::make_unique<cv::Mat>(std::move(fmat)));
}

absl::Status SaveFloatImagePng(const Image& image,
                               const std::string& png_file_path, double scale,
                               double shift, ImageRotation rotation) {
  // The convertTo function supports a scale/shift (alpha/beta args) which is
  // supposed to use saturate_cast, but it doesn't work as expected when an
  // overflow occurs. Apply the scale/shift and clip to 16bit range manually.
  cv::Mat scaled = image.data() * scale + shift;
  scaled.forEach<float>([](float& p, const int* position) -> void {
    p = cv::saturate_cast<ushort>(p);
  });

  cv::Mat mat;
  scaled.convertTo(mat, CV_16UC1);

  ApplyRotation(mat, mat, rotation);
  return cv::imwrite(png_file_path, mat)
             ? absl::OkStatus()
             : absl::InternalError(absl::StrFormat("Failed to save image to %s",
                                                   png_file_path));
}

}  // namespace guideline::util
