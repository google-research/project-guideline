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

#include <numeric>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/file.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/image_io.h"

namespace guideline::util {
namespace {

const char kTestDataDir[] =
    "/project_guideline/project_guideline/util/testdata/";

static constexpr uint64_t kTimestampNs = 123456789;

static constexpr size_t kImageWidth = 640;
static constexpr size_t kImageHeight = 480;
static constexpr size_t kYuvNumPlanes = 3;

static constexpr size_t kI420PlaneSize[kYuvNumPlanes] = {307200, 76800, 76800};
static constexpr size_t kI420RowStride[kYuvNumPlanes] = {640, 320, 320};
static constexpr size_t kI420PixelStride[kYuvNumPlanes] = {1, 1, 1};

static constexpr size_t kNVPlaneSize[kYuvNumPlanes] = {307200, 153599, 153599};
static constexpr size_t kNVRowStride[kYuvNumPlanes] = {640, 640, 640};
static constexpr size_t kNVPixelStride[kYuvNumPlanes] = {1, 2, 2};

std::string GetTestDataPath(const std::string& filename) {
  return std::string(getenv("TEST_SRCDIR")) + kTestDataDir + filename;
}

// The RGB <-> YUV conversions are lossy so this utility checks the average
// difference between pixel values to determine if 2 images are roughly
// equivalent.
bool ImagesAlmostEqual(const Image& a, const Image& b, double tolerance = 1.5) {
  cv::Mat diff = cv::abs(a.data() - b.data());
  double avg_value_diff =
      1.0 * std::accumulate(diff.begin<uint8_t>(), diff.end<uint8_t>(), 0) /
      diff.total();
  return avg_value_diff < tolerance;
}

TEST(ImageUtil, ConvertYuvToRgbI420NonInterleaved) {
  GL_ASSERT_OK_AND_ASSIGN(
      auto golden_image,
      util::LoadImage(GetTestDataPath("landscape.png"), /*keep_alpha=*/false));

  std::string i420_data_str;
  GL_ASSERT_OK(util::ReadFileToString(GetTestDataPath("landscape_i420.bin"),
                                      i420_data_str));

  ASSERT_EQ(i420_data_str.size(),
            kI420PlaneSize[0] + kI420PlaneSize[1] + kI420PlaneSize[2]);

  const uint8_t* data = reinterpret_cast<const uint8_t*>(i420_data_str.c_str());

  std::vector<ImagePlane> planes(kYuvNumPlanes);
  for (int i = 0; i < kYuvNumPlanes; ++i) {
    planes[i].row_stride = kI420RowStride[i];
    planes[i].pixel_stride = kI420PixelStride[i];
    planes[i].data_length = kI420PlaneSize[i];
  }

  planes[0].data = data;
  planes[1].data = data + kI420PlaneSize[0];
  planes[2].data = planes[1].data + kI420PlaneSize[1];

  GL_ASSERT_OK_AND_ASSIGN(auto rgb, ConvertYuvToRgb(kImageWidth, kImageHeight,
                                                    planes, kTimestampNs));

  EXPECT_TRUE(ImagesAlmostEqual(rgb, golden_image));
}

TEST(ImageUtil, ConvertYuvToRgbNV21) {
  GL_ASSERT_OK_AND_ASSIGN(
      auto golden_image,
      util::LoadImage(GetTestDataPath("landscape.png"), /*keep_alpha=*/false));

  std::string nv21_data_str;
  GL_ASSERT_OK(util::ReadFileToString(GetTestDataPath("landscape_nv21.bin"),
                                      nv21_data_str));

  ASSERT_EQ(nv21_data_str.size(),
            kNVPlaneSize[0] + kNVPlaneSize[1] + kNVPlaneSize[2]);

  const uint8_t* data = reinterpret_cast<const uint8_t*>(nv21_data_str.c_str());

  std::vector<ImagePlane> planes(kYuvNumPlanes);
  for (int i = 0; i < kYuvNumPlanes; ++i) {
    planes[i].row_stride = kNVRowStride[i];
    planes[i].pixel_stride = kNVPixelStride[i];
    planes[i].data_length = kNVPlaneSize[i];
  }

  planes[0].data = data;
  planes[2].data = data + kNVPlaneSize[0];
  planes[1].data = planes[2].data + 1;

  GL_ASSERT_OK_AND_ASSIGN(auto rgb, ConvertYuvToRgb(kImageWidth, kImageHeight,
                                                    planes, kTimestampNs));

  EXPECT_TRUE(ImagesAlmostEqual(rgb, golden_image));
}

TEST(ImageUtil, ConvertYuvToRgbNV12) {
  GL_ASSERT_OK_AND_ASSIGN(
      auto golden_image,
      util::LoadImage(GetTestDataPath("landscape.png"), /*keep_alpha=*/false));

  std::string nv21_data_str;
  GL_ASSERT_OK(util::ReadFileToString(GetTestDataPath("landscape_nv12.bin"),
                                      nv21_data_str));

  ASSERT_EQ(nv21_data_str.size(),
            kNVPlaneSize[0] + kNVPlaneSize[1] + kNVPlaneSize[2]);

  const uint8_t* data = reinterpret_cast<const uint8_t*>(nv21_data_str.c_str());

  std::vector<ImagePlane> planes(kYuvNumPlanes);
  for (int i = 0; i < kYuvNumPlanes; ++i) {
    planes[i].row_stride = kNVRowStride[i];
    planes[i].pixel_stride = kNVPixelStride[i];
    planes[i].data_length = kNVPlaneSize[i];
  }

  planes[0].data = data;
  // NV12 is the same as NV21 but with swapped UV planes.
  planes[1].data = data + kNVPlaneSize[0];
  planes[2].data = planes[1].data + 1;

  GL_ASSERT_OK_AND_ASSIGN(auto rgb, ConvertYuvToRgb(kImageWidth, kImageHeight,
                                                    planes, kTimestampNs));

  EXPECT_TRUE(ImagesAlmostEqual(rgb, golden_image));
}

}  // namespace
}  // namespace guideline::util
