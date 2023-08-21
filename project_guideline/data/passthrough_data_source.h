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

#ifndef PROJECT_GUIDELINE_DATA_PASSTHROUGH_DATA_SOURCE_H_
#define PROJECT_GUIDELINE_DATA_PASSTHROUGH_DATA_SOURCE_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "project_guideline/data/data_source.h"
#include "project_guideline/util/image.h"

namespace guideline::data {

class PassthroughDataEmitter {
 public:
  explicit PassthroughDataEmitter() = default;
  void OnImage(const std::shared_ptr<const util::Image>& image);

 private:
  friend class PassthroughDataSource;
  void SetCallbacks(const DataSource::ImageCallback& callback);
  void ClearCallbacks();

  absl::Mutex mutex_;
  std::optional<DataSource::ImageCallback> image_callback_
      ABSL_GUARDED_BY(mutex_);
};

class PassthroughDataSource : public DataSource {
 public:
  explicit PassthroughDataSource(
      std::shared_ptr<PassthroughDataEmitter> emitter);

  absl::Status Start() override;
  absl::Status Stop() override;

  absl::Status AddImageCallback(const ImageCallback& callback) override;
  absl::Status AddOnCompletedCallback(
      const OnCompletedCallback& callback) override;

  absl::Status SetGlTextureOutput(
      const GlTextureCallback& gl_texture_callback) override;
  absl::Status GlAttachTexture(int32_t texture_id) override;
  absl::Status GlUpdateTexture() override;

 private:
  void OnImage(const std::shared_ptr<const util::Image>& image);

 private:
  std::shared_ptr<PassthroughDataEmitter> emitter_;
  bool started_ ABSL_GUARDED_BY(mutex_) = false;
  std::vector<ImageCallback> image_callbacks_ ABSL_GUARDED_BY(mutex_);
  absl::Mutex mutex_;
};

}  // namespace guideline::data

#endif  // PROJECT_GUIDELINE_DATA_PASSTHROUGH_DATA_SOURCE_H_
