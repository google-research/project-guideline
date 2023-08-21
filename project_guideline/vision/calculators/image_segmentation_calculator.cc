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

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/port/ret_check.h"
#include "mediapipe/tasks/cc/vision/image_segmenter/image_segmenter.h"
#include "project_guideline/util/status.h"
#include "project_guideline/vision/calculators/image_segmentation_calculator_options.pb.h"

namespace guideline::vision {

namespace {
using ::mediapipe::CalculatorBase;
using ::mediapipe::CalculatorContext;
using ::mediapipe::CalculatorContract;
using ::mediapipe::ImageFormat;
using ::mediapipe::ImageFrame;
using mediapipe::tasks::core::BaseOptions;
using mediapipe::tasks::vision::image_segmenter::ImageSegmenter;
using mediapipe::tasks::vision::image_segmenter::ImageSegmenterOptions;

constexpr char kImageTag[] = "IMAGE";
constexpr char kMaskTag[] = "MASK";
}  // namespace

class ImageSegmentationCalculator : public CalculatorBase {
 public:
  ImageSegmentationCalculator() = default;
  ~ImageSegmentationCalculator() override = default;

  static absl::Status GetContract(CalculatorContract* cc);

  absl::Status Open(CalculatorContext* cc) override;
  absl::Status Process(CalculatorContext* cc) override;

 private:
  std::unique_ptr<ImageSegmenter> image_segmenter_;
};
REGISTER_CALCULATOR(ImageSegmentationCalculator);

absl::Status ImageSegmentationCalculator::GetContract(CalculatorContract* cc) {
  RET_CHECK(cc->Inputs().HasTag(kImageTag));

  cc->Inputs().Tag(kImageTag).Set<ImageFrame>();
  cc->Outputs().Tag(kMaskTag).Set<ImageFrame>();
  return absl::OkStatus();
}

absl::Status ImageSegmentationCalculator::Open(CalculatorContext* cc) {
  const auto& options = cc->Options<ImageSegmentationCalculatorOptions>();

  auto segmenter_options = std::make_unique<ImageSegmenterOptions>();

  switch (options.model_file().type_case()) {
    case ImageSegmentationCalculatorOptions::ModelFile::TypeCase::kContent:
      segmenter_options->base_options.model_asset_buffer =
          std::make_unique<std::string>(options.model_file().content());
      break;
    case ImageSegmentationCalculatorOptions::ModelFile::TypeCase::kFilePath:
      segmenter_options->base_options.model_asset_path =
          options.model_file().file_path();
      break;
    case ImageSegmentationCalculatorOptions::ModelFile::TypeCase::
        kFileDescriptor:
      segmenter_options->base_options.model_asset_descriptor_meta.fd =
          options.model_file().file_descriptor().fd();
      segmenter_options->base_options.model_asset_descriptor_meta.length =
          options.model_file().file_descriptor().length();
      segmenter_options->base_options.model_asset_descriptor_meta.offset =
          options.model_file().file_descriptor().offset();
      break;
    case ImageSegmentationCalculatorOptions::ModelFile::TypeCase::
        kContentPointer:
      segmenter_options->base_options.model_asset_buffer =
          std::make_unique<std::string>(
              reinterpret_cast<char*>(
                  options.model_file().content_pointer().pointer()),
              options.model_file().content_pointer().length());
      break;
    case ImageSegmentationCalculatorOptions::ModelFile::TypeCase::TYPE_NOT_SET:
      RET_CHECK_FAIL() << "ModelFile must be specified";
  }

  segmenter_options->base_options.delegate =
      BaseOptions::Delegate::EDGETPU_NNAPI;

  GL_ASSIGN_OR_RETURN(image_segmenter_,
                      ImageSegmenter::Create(std::move(segmenter_options)));

  return absl::OkStatus();
}

absl::Status ImageSegmentationCalculator::Process(CalculatorContext* cc) {
  const auto& options = cc->Options<ImageSegmentationCalculatorOptions>();
  const auto& input_img = cc->Inputs().Tag(kImageTag).Get<ImageFrame>();

  // The MediaPipe tasks API requires an Image rather than ImageFrame even
  // though the Image is treated as immutable. Rather than copy the image data,
  // make a non-const ImageFrame view of the input ImageFrame.
  auto image_frame = std::make_unique<ImageFrame>(
      input_img.Format(), input_img.Width(), input_img.Height(),
      input_img.WidthStep(), const_cast<uint8_t*>(input_img.PixelData()),
      ImageFrame::PixelDataDeleter::kNone);

  ASSIGN_OR_RETURN(
      auto segment_result,
      image_segmenter_->Segment(mediapipe::Image(std::move(image_frame))));

  RET_CHECK(segment_result.confidence_masks.has_value())
      << "Missing confidence masks";
  RET_CHECK_GT(segment_result.confidence_masks->size(),
               options.output_mask_index())
      << "Invalid confidence mask index";

  const mediapipe::Image& mask =
      segment_result.confidence_masks->at(options.output_mask_index());

  std::shared_ptr<mediapipe::ImageFrame> mask_frame =
      mask.GetImageFrameSharedPtr();
  RET_CHECK_EQ(mask_frame->Format(), mediapipe::ImageFormat::VEC32F1)
      << "Mask output must be VEC32F1";

  // Create an ImageFrame which wraps the data from the mask result to avoid
  // copying the data.
  auto output_mask = std::make_unique<ImageFrame>(
      mask_frame->Format(), mask_frame->Width(), mask_frame->Height(),
      mask_frame->WidthStep(), const_cast<uint8_t*>(mask_frame->PixelData()),
      [mask_frame](uint8_t* unused_data) {
        // Lambda will retain the mask_frame shared_ptr so the underlying data
        // is not released before this ImageFrame.
      });

  cc->Outputs().Tag(kMaskTag).Add(output_mask.release(), cc->InputTimestamp());

  return absl::OkStatus();
}

}  // namespace guideline::vision
