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

#ifndef PROJECT_GUIDELINE_UTIL_IMAGE_IO_H_
#define PROJECT_GUIDELINE_UTIL_IMAGE_IO_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "project_guideline/util/image.h"

namespace guideline::util {

absl::StatusOr<Image> LoadImage(const std::string& file_path,
                                bool keep_alpha = true,
                                ImageRotation rotation = ImageRotation::kNone);

absl::Status SaveImage(const Image& image, const std::string& file_path,
                       ImageRotation rotation = ImageRotation::kNone);

// Loads a float image from a 16-bit grayscale PNG.
// The scale and shift parameters should match those used with
// SaveFloatImagePng().
// The PNG UINT16 values will be converted to float using the inverse of the
// given scale/shift parameters.
absl::StatusOr<TypedImage<ImageFormat::kFP32, float>> LoadFloatImagePng(
    const std::string& png_file_path, double scale = 1., double shift = 0.,
    ImageRotation rotation = ImageRotation::kNone);

// Saves a float image as a 16-bit grayscale PNG.
// The float values will be converted to UINT16 using the given scale and
// shift parameters, and the resulting values will be clipped in the UNIT16
// range [0, 65535].
absl::Status SaveFloatImagePng(const Image& image,
                               const std::string& png_file_path,
                               double scale = 1., double shift = 0.,
                               ImageRotation rotation = ImageRotation::kNone);

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_IMAGE_IO_H_
