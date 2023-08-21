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

#ifndef PROJECT_GUIDELINE_DATA_DATA_SOURCE_H_
#define PROJECT_GUIDELINE_DATA_DATA_SOURCE_H_

#include <functional>
#include <memory>

#include "absl/status/status.h"
#include "project_guideline/util/image.h"

namespace guideline::data {

class DataSource {
 public:
  using OnCompletedCallback = std::function<void(absl::Status)>;
  using ImageCallback =
      std::function<void(const std::shared_ptr<const util::Image>&)>;
  using GlTextureCallback = std::function<void()>;

  virtual ~DataSource() = default;

  virtual absl::Status Start() = 0;
  virtual absl::Status Stop() = 0;

  virtual absl::Status AddImageCallback(const ImageCallback& callback) = 0;
  virtual absl::Status AddOnCompletedCallback(
      const OnCompletedCallback& callback) = 0;

  virtual absl::Status SetGlTextureOutput(
      const GlTextureCallback& gl_texture_callback) = 0;
  virtual absl::Status GlAttachTexture(int32_t texture_id) = 0;
  virtual absl::Status GlUpdateTexture() = 0;
};

}  // namespace guideline::data

#endif  // PROJECT_GUIDELINE_DATA_DATA_SOURCE_H_
