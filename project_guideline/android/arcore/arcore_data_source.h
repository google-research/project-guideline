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

#ifndef PROJECT_GUIDELINE_ANDROID_ARCORE_ARCORE_DATA_SOURCE_H_
#define PROJECT_GUIDELINE_ANDROID_ARCORE_ARCORE_DATA_SOURCE_H_

#include <optional>

#include "absl/synchronization/mutex.h"
#include "project_guideline/android/arcore/arcore_session.h"
#include "project_guideline/data/data_source.h"

namespace guideline::arcore {

// DataSource implementation that forwards images from the ARCoreSession.
class ArcoreDataSource : public data::DataSource {
 public:
  explicit ArcoreDataSource(std::shared_ptr<ArcoreSession> session);

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
  void OnFrameData(const ArcoreFrameData& frame_data);

  std::shared_ptr<ArcoreSession> session_;

  absl::Mutex mutex_;
  bool started_ ABSL_GUARDED_BY(mutex_) = false;
  std::optional<int> session_callback_key_ ABSL_GUARDED_BY(mutex_) =
      std::nullopt;
  std::vector<ImageCallback> image_callbacks_ ABSL_GUARDED_BY(mutex_);
};

}  // namespace guideline::arcore

#endif  // PROJECT_GUIDELINE_ANDROID_ARCORE_ARCORE_DATA_SOURCE_H_
