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

#include "project_guideline/util/mediapipe.h"

#include "absl/log/check.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/image_io.h"
#include "project_guideline/util/status.h"

namespace guideline::util {

namespace {
using mediapipe::formats::MatView;
}  // namespace

absl::StatusOr<mediapipe::ImageFrame> LoadImageFrame(
    const std::string& file_path) {
  GL_ASSIGN_OR_RETURN(auto image,
                      util::LoadImage(file_path, /*keep_alpha=*/false));
  mediapipe::ImageFrame image_frame;
  image_frame.CopyPixelData(
      mediapipe::ImageFormat::SRGB, image.width(), image.height(),
      image.width() * image.num_channels(), image.data().ptr<uint8_t>(),
      mediapipe::ImageFrame::kDefaultAlignmentBoundary);
  return image_frame;
}

Image CopyImageFrameToImage(const mediapipe::ImageFrame& frame) {
  ImageFormat format;
  switch (frame.Format()) {
    case mediapipe::ImageFormat::SRGB:
      format = ImageFormat::kRGB;
      break;
    case mediapipe::ImageFormat::SRGBA:
      format = ImageFormat::kRGBA;
      break;
    case mediapipe::ImageFormat::VEC32F1:
      format = ImageFormat::kFP32;
      break;
    case mediapipe::ImageFormat::GRAY8:
      format = ImageFormat::kUINT8;
      break;
    case mediapipe::ImageFormat::GRAY16:
      format = ImageFormat::kUINT16;
      break;
    default:
      CHECK(false) << "Unsupported image format: "
                   << static_cast<int>(frame.Format());
  }
  return Image(format, frame.Width(), frame.Height(),
               std::make_unique<cv::Mat>(MatView(&frame).clone()));
}

mediapipe::ImageFrame CopyImageToImageFrame(const util::Image& image) {
  mediapipe::ImageFormat::Format format;
  switch (image.format()) {
    case ImageFormat::kRGB:
      format = mediapipe::ImageFormat::SRGB;
      break;
    case ImageFormat::kRGBA:
      format = mediapipe::ImageFormat::SRGBA;
      break;
    case ImageFormat::kFP32:
      format = mediapipe::ImageFormat::VEC32F1;
      break;
    case ImageFormat::kUINT8:
      format = mediapipe::ImageFormat::GRAY8;
      break;
    case ImageFormat::kUINT16:
      format = mediapipe::ImageFormat::GRAY16;
      break;
    case ImageFormat::kEmpty:
      format = mediapipe::ImageFormat::UNKNOWN;
      break;
  }
  mediapipe::ImageFrame image_frame;
  image_frame.CopyPixelData(format, image.width(), image.height(),
                            image.data().ptr(),
                            mediapipe::ImageFrame::kDefaultAlignmentBoundary);
  return image_frame;
}

}  // namespace guideline::util
