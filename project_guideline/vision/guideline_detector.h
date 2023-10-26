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

#ifndef PROJECT_GUIDELINE_VISION_GUIDELINE_DETECTOR_H_
#define PROJECT_GUIDELINE_VISION_GUIDELINE_DETECTOR_H_

#include <memory>
#include <thread>  // NOLINT(build/c++11)

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "Eigen/Core"
#include "mediapipe/tasks/cc/vision/image_segmenter/image_segmenter.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/image.h"

namespace guideline::vision {

// Runs ML segmentation pipeline to detect the Guideline and depth from an
// image frame.
class GuidelineDetector {
 public:
  using DetectionCallback = std::function<void(
      const int64_t timestamp_us, const std::vector<Eigen::Vector3f>& keypoints,
      std::shared_ptr<const util::ConfidenceMask> guideline_mask,
      std::shared_ptr<const util::DepthImage> depth_map)>;

  static absl::StatusOr<std::unique_ptr<GuidelineDetector>> Create(
      const GuidelineEngineConfig_DetectorOptions& options);

  ~GuidelineDetector();

  absl::Status Start();
  absl::Status Stop();

  void OnImage(const std::shared_ptr<const util::Image>& image);

  void AddCallback(const DetectionCallback& callback)
      ABSL_LOCKS_EXCLUDED(callbacks_mutex_);

 private:
  GuidelineDetector(
      const GuidelineEngineConfig_DetectorOptions& options,
      std::unique_ptr<mediapipe::tasks::vision::image_segmenter::ImageSegmenter>
          depth_segmenter,
      std::unique_ptr<mediapipe::tasks::vision::image_segmenter::ImageSegmenter>
          guideline_segmenter);

  void RunDetector();

  absl::Status ProcessImage(const util::Image& image);

  const GuidelineEngineConfig_DetectorOptions options_;
  std::unique_ptr<mediapipe::tasks::vision::image_segmenter::ImageSegmenter>
      depth_segmenter_;
  std::unique_ptr<mediapipe::tasks::vision::image_segmenter::ImageSegmenter>
      guideline_segmenter_;

  absl::Mutex image_mutex_;
  std::shared_ptr<const util::Image> last_image_ ABSL_GUARDED_BY(image_mutex_) =
      nullptr;

  std::atomic<bool> running_ = false;
  std::unique_ptr<std::thread> detector_thread_;

  absl::Mutex callbacks_mutex_;
  std::vector<DetectionCallback> callbacks_ ABSL_GUARDED_BY(callbacks_mutex_);
};

}  // namespace guideline::vision

#endif  // PROJECT_GUIDELINE_VISION_GUIDELINE_DETECTOR_H_
