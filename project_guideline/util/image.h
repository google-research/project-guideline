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

#ifndef PROJECT_GUIDELINE_UTIL_IMAGE_H_
#define PROJECT_GUIDELINE_UTIL_IMAGE_H_

#include <cstdint>
#include <memory>

#include <opencv2/core.hpp>  // keep include

namespace guideline::util {

enum class ImageFormat {
  // Empty image with 0 pixels.
  kEmpty,

  // RGB with each pixel as 3bytes.
  kRGB,

  // RGBA with each pixel as 4bytes.
  kRGBA,

  // Unsigned int, 1 byte per pixel.
  kUINT8,

  // Unsigned int, 2 bytes per pixel.
  kUINT16,

  // 32-bit floating point, 4 bytes per pixel.
  kFP32
};

// Metadata associated with this image.
struct ImageMetadata {
  uint64_t timestamp_ns = 0;
};

class Image {
 public:
  // Interface for data which is scoped to an Image instance.
  // For example the cv:Mat may wrap data owned by another object which should
  // not be deallocated until the Image is deleted. In that case ScopedData
  // may be subclassed with a smart pointer member, or a destructor, which will
  // delete the backing data.
  class ScopedData {
   public:
    virtual ~ScopedData() = default;
  };

  // Creates an empty image of type Format::kEmpty.
  Image();

  // Creates an image from the given cv::Mat.
  Image(ImageFormat format, uint16_t width, uint16_t height,
        std::unique_ptr<const cv::Mat> data, const ImageMetadata& metadata = {},
        std::unique_ptr<ScopedData> scoped_data = nullptr);

  inline ImageFormat format() const { return format_; }
  inline uint16_t width() const { return width_; }
  inline uint16_t height() const { return height_; }
  inline int num_channels() const { return data_->channels(); }

  const cv::Mat& data() const { return *data_; }

  const ImageMetadata& metadata() const { return metadata_; }

  ImageMetadata& mutable_metadata() { return metadata_; }

 private:
  const ImageFormat format_;
  const uint16_t width_;
  const uint16_t height_;
  std::unique_ptr<const cv::Mat> data_;
  ImageMetadata metadata_;
  std::unique_ptr<ScopedData> scoped_data_ = nullptr;
};

template <enum ImageFormat Format, typename DataType, typename Enabled = void>
class TypedImage;

template <enum ImageFormat Format, typename DataType>
class TypedImage<Format, DataType> : public Image {
 public:
  TypedImage(uint16_t width, uint16_t height,
             std::unique_ptr<const cv::Mat> data,
             const ImageMetadata& metadata = {},
             std::unique_ptr<ScopedData> scoped_data = nullptr)
      : Image(Format, width, height, std::move(data), metadata,
              std::move(scoped_data)) {}

  inline DataType at(int y, int x) const {
    return data().template at<DataType>(y, x);
  }

  inline DataType operator()(int y, int x) const {
    return data().template at<DataType>(y, x);
  }

  TypedImage<Format, DataType> Clone() const {
    return TypedImage<Format, DataType>(
        width(), height(), std::make_unique<cv::Mat>(data().clone()));
  }
};

typedef TypedImage<ImageFormat::kFP32, float> DepthImage;
typedef TypedImage<ImageFormat::kFP32, float> ConfidenceMask;
typedef TypedImage<ImageFormat::kUINT16, uint16_t> DepthImageU16;
typedef TypedImage<ImageFormat::kUINT8, uint8_t> ConfidenceImageU8;
typedef TypedImage<ImageFormat::kRGB, cv::Vec3b> RGBImage;
typedef TypedImage<ImageFormat::kRGBA, cv::Vec4b> RGBAImage;

Image RGBAToRGB(const Image& rgba);

// Specifies image rotation (in degrees, clockwise).
enum class ImageRotation { kNone, k90, k180, k270 };

// The expected rotation of camera images for Android device in
// reverse-landscape orientation.
constexpr ImageRotation kReverseLandscapeRotation = ImageRotation::k180;
constexpr int kReverseLandscapeCvRotateCode = cv::ROTATE_180;

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_IMAGE_H_
