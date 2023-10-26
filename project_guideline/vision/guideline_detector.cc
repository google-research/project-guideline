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

#include "project_guideline/vision/guideline_detector.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>  // keep include
#include "absl/functional/bind_front.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "mediapipe/framework/formats/image.h"
#include "mediapipe/framework/formats/image_format.pb.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/tasks/cc/core/base_options.h"
#include "mediapipe/tasks/cc/vision/image_segmenter/image_segmenter.h"
#include "mediapipe/tasks/cc/vision/image_segmenter/image_segmenter_result.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/embedded_file_toc.h"
#include "project_guideline/util/image.h"
#include "project_guideline/util/status.h"
#include "project_guideline/vision/keypoint_extraction.h"
#include "project_guideline/vision/models/depth_tflite_embed.h"
#include "project_guideline/vision/models/guideline_tflite_embed.h"

namespace guideline::vision {

namespace {
using mediapipe::tasks::core::BaseOptions;
using mediapipe::tasks::vision::image_segmenter::ImageSegmenter;
using mediapipe::tasks::vision::image_segmenter::ImageSegmenterOptions;
using mediapipe::tasks::vision::image_segmenter::ImageSegmenterResult;
using util::EmbeddedFileToc;

inline constexpr bool kDepthOutputInverse = true;
inline constexpr float kDepthOutputScale = 1.512;
inline constexpr float kDepthOutputShift = 0.117;

// ScopedData for Images that wrap a MediaPipe ImageFrame.
class MediaPipeScopedData : public util::Image::ScopedData {
 public:
  explicit MediaPipeScopedData(
      std::shared_ptr<mediapipe::ImageFrame> mediapipe_frame)
      : mediapipe_frame_(std::move(mediapipe_frame)) {}

 private:
  std::shared_ptr<mediapipe::ImageFrame> mediapipe_frame_;
};

std::unique_ptr<std::string> FileTocToString(const EmbeddedFileToc& toc) {
  return std::make_unique<std::string>(toc.data, toc.size);
}

void ConfigureSegmenter(ImageSegmenterOptions& segmenter_options,
                        std::unique_ptr<std::string> model) {
  segmenter_options.base_options.delegate =
      BaseOptions::Delegate::EDGETPU_NNAPI;
  segmenter_options.base_options.model_asset_buffer = std::move(model);
}
}  // namespace

absl::StatusOr<std::unique_ptr<GuidelineDetector>> GuidelineDetector::Create(
    const GuidelineEngineConfig_DetectorOptions& options) {
  auto depth_segmenter_options = std::make_unique<ImageSegmenterOptions>();
  ConfigureSegmenter(*depth_segmenter_options,
                     FileTocToString(depth_tflite_embed_create()[0]));
  GL_ASSIGN_OR_RETURN(
      auto depth_segmenter,
      ImageSegmenter::Create(std::move(depth_segmenter_options)));

  auto guideline_segmenter_options = std::make_unique<ImageSegmenterOptions>();
  ConfigureSegmenter(*guideline_segmenter_options,
                     FileTocToString(guideline_tflite_embed_create()[0]));

  GL_ASSIGN_OR_RETURN(
      auto guideline_segmenter,
      ImageSegmenter::Create(std::move(guideline_segmenter_options)));

  return absl::WrapUnique(new GuidelineDetector(
      options, std::move(depth_segmenter), std::move(guideline_segmenter)));
}

GuidelineDetector::GuidelineDetector(
    const GuidelineEngineConfig_DetectorOptions& options,
    std::unique_ptr<ImageSegmenter> depth_segmenter,
    std::unique_ptr<ImageSegmenter> guideline_segmenter)
    : options_(options),
      depth_segmenter_(std::move(depth_segmenter)),
      guideline_segmenter_(std::move(guideline_segmenter)) {}

GuidelineDetector::~GuidelineDetector() {
  if (detector_thread_) {
    auto status = Stop();
    if (!status.ok()) {
      LOG(ERROR) << "Failed to stop detector: " << status;
    }
  }
}

absl::Status GuidelineDetector::Start() {
  CHECK(detector_thread_ == nullptr) << "Already started";

  running_ = true;
  detector_thread_ = std::make_unique<std::thread>(
      absl::bind_front(&GuidelineDetector::RunDetector, this));

  return absl::OkStatus();
}
absl::Status GuidelineDetector::Stop() {
  CHECK(detector_thread_) << "Already stopped";
  running_ = false;

  {
    absl::MutexLock lock(&image_mutex_);
    last_image_ = nullptr;
  }

  detector_thread_->join();
  detector_thread_ = nullptr;

  CHECK_OK(guideline_segmenter_->Close());
  CHECK_OK(depth_segmenter_->Close());

  return absl::OkStatus();
}

void GuidelineDetector::AddCallback(const DetectionCallback& callback) {
  absl::MutexLock lock(&callbacks_mutex_);
  callbacks_.push_back(callback);
}

void GuidelineDetector::OnImage(
    const std::shared_ptr<const util::Image>& image) {
  if (running_) {
    absl::MutexLock lock(&image_mutex_);
    last_image_ = image;
  }
}

void GuidelineDetector::RunDetector() {
  while (running_) {
    std::shared_ptr<const util::Image> image;
    image_mutex_.Lock();
    auto has_image_or_stopped =
        [this]() ABSL_SHARED_LOCKS_REQUIRED(image_mutex_) {
          return !running_ || last_image_ != nullptr;
        };
    image_mutex_.Await(absl::Condition(&has_image_or_stopped));
    image = last_image_;
    last_image_ = nullptr;
    image_mutex_.Unlock();

    if (image) {
      auto status = ProcessImage(*image);
      if (!status.ok()) {
        LOG(ERROR) << "Failed to process image: " << status;
      }
    }
  }
}

absl::Status GuidelineDetector::ProcessImage(const util::Image& image) {
  auto image_frame = std::make_shared<mediapipe::ImageFrame>();
  image_frame->CopyPixelData(mediapipe::ImageFormat::SRGB, image.width(),
                             image.height(), image.data().ptr<uint8_t>(),
                             mediapipe::ImageFrame::kDefaultAlignmentBoundary);

  // Rotate the image to natural orientation
  // (assuming reverse landscape camera image which is rotated 180 degrees).
  // Note: this could be done by specifying rotation_degrees in
  // mediapipe::tasks::vision::core::ImageProcessingOptions with the
  // ImageSegmenter, however since 2 ImageSegmenters are being used it is
  // more efficient to rotate once upfront.
  cv::Mat mat = mediapipe::formats::MatView(image_frame.get());
  cv::rotate(mat, mat, util::kReverseLandscapeCvRotateCode);

  mediapipe::Image mp_image(image_frame);

  GL_ASSIGN_OR_RETURN(ImageSegmenterResult depth_result,
                      depth_segmenter_->Segment(mp_image));
  CHECK(depth_result.confidence_masks.has_value() &&
        depth_result.confidence_masks->size() == 1)
      << "Invalid depth segmentation result";

  GL_ASSIGN_OR_RETURN(ImageSegmenterResult guideline_result,
                      guideline_segmenter_->Segment(mp_image));
  CHECK(guideline_result.confidence_masks.has_value() &&
        guideline_result.confidence_masks->size() == 2)
      << "Invalid guideline segmentation result";

  mediapipe::Image& depth_mask = depth_result.confidence_masks->at(0);

  // The segmentation has 2 channels. confidence_masks[0] is the background
  // segmentation and confidence_masks[1] is the line. Currently these are
  // pure inverses of each other.
  mediapipe::Image& guideline_mask = guideline_result.confidence_masks->at(1);

  CHECK_EQ(depth_mask.GetImageFrameSharedPtr()->Format(),
           mediapipe::ImageFormat::VEC32F1)
      << "Depth output must be VEC32F1";

  CHECK_EQ(guideline_mask.GetImageFrameSharedPtr()->Format(),
           mediapipe::ImageFormat::VEC32F1)
      << "Mask output must be VEC32F1";

  auto depth_mat = std::make_unique<cv::Mat>(
      mediapipe::formats::MatView(depth_mask.GetImageFrameSharedPtr().get()));

  // Rotate the depth result to match the input image.
  cv::rotate(*depth_mat, *depth_mat, util::kReverseLandscapeCvRotateCode);

  // Convert raw to metric depth.
  if (kDepthOutputInverse || kDepthOutputScale != 1.0 ||
      kDepthOutputShift != 0.0) {
    depth_mat->forEach<float>([](float& p, const int* position) -> void {
      p = std::max(0.0f, p * kDepthOutputScale + kDepthOutputShift);
      if (kDepthOutputInverse) {
        p = 1.0f / p;
      }
    });
  }

  auto depth_image = std::make_shared<util::DepthImage>(
      depth_mask.width(), depth_mask.height(), std::move(depth_mat),
      util::ImageMetadata(),
      std::make_unique<MediaPipeScopedData>(
          depth_mask.GetImageFrameSharedPtr()));

  auto mask_mat = std::make_unique<cv::Mat>(mediapipe::formats::MatView(
      guideline_mask.GetImageFrameSharedPtr().get()));
  // Rotate the mask to match the input image.
  cv::rotate(*mask_mat, *mask_mat, util::kReverseLandscapeCvRotateCode);

  // The keypoints will be ordered from top of image to bottom. Since the
  // mask is rotated 180 in reverse landscape, the keypoints will be ordered
  // from closest to farthest from user.
  GL_ASSIGN_OR_RETURN(std::vector<Eigen::Vector3f> keypoints,
                      ExtractLineKeypointsFromMask(
                          *mask_mat, options_.keypoint_extractor_options()));

  auto mask_image = std::make_shared<util::ConfidenceMask>(
      guideline_mask.width(), guideline_mask.height(), std::move(mask_mat),
      util::ImageMetadata(),
      std::make_unique<MediaPipeScopedData>(
          guideline_mask.GetImageFrameSharedPtr()));

  absl::MutexLock lock(&callbacks_mutex_);
  for (const auto& callback : callbacks_) {
    int64_t timestamp_us = absl::ToInt64Microseconds(
        absl::Nanoseconds(image.metadata().timestamp_ns));
    callback(timestamp_us, keypoints, mask_image, depth_image);
  }

  return absl::OkStatus();
}

}  // namespace guideline::vision
