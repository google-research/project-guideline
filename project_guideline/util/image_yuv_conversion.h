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

#ifndef PROJECT_GUIDELINE_UTIL_IMAGE_YUV_CONVERSION_H_
#define PROJECT_GUIDELINE_UTIL_IMAGE_YUV_CONVERSION_H_

#include "absl/status/statusor.h"
#include "project_guideline/util/image.h"

namespace guideline::util {

// Represents a single plane of an image.
struct ImagePlane {
  int32_t pixel_stride;
  int32_t row_stride;
  // Pointer to the plane data, which is not owned by this struct.
  const uint8_t* data;
  int32_t data_length;
};

// Converts a YUV image into an RGB util::Image.
absl::StatusOr<Image> ConvertYuvToRgb(int image_width, int image_height,
                                      const std::vector<ImagePlane>& planes,
                                      uint64_t image_timestamp_ns);

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_IMAGE_YUV_CONVERSION_H_
