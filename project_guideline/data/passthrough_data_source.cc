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

#include "project_guideline/data/passthrough_data_source.h"

#include <memory>
#include <optional>

#include "absl/functional/bind_front.h"
#include "absl/status/status.h"

namespace guideline::data {

void PassthroughDataEmitter::OnImage(
    const std::shared_ptr<const util::Image>& image) {
  std::optional<DataSource::ImageCallback> callback;
  {
    absl::MutexLock lock(&mutex_);
    callback = image_callback_;
  }
  if (callback.has_value()) {
    (*callback)(image);
  }
}

void PassthroughDataEmitter::SetCallbacks(
    const DataSource::ImageCallback& callback) {
  absl::MutexLock lock(&mutex_);
  image_callback_ = callback;
}

void PassthroughDataEmitter::ClearCallbacks() {
  absl::MutexLock lock(&mutex_);
  image_callback_ = std::nullopt;
}

PassthroughDataSource::PassthroughDataSource(
    std::shared_ptr<PassthroughDataEmitter> emitter)
    : emitter_(emitter) {}

absl::Status PassthroughDataSource::Start() {
  absl::MutexLock lock(&mutex_);
  started_ = true;
  emitter_->SetCallbacks(
      absl::bind_front(&PassthroughDataSource::OnImage, this));
  return absl::OkStatus();
}
absl::Status PassthroughDataSource::Stop() {
  absl::MutexLock lock(&mutex_);
  started_ = false;
  emitter_->ClearCallbacks();
  return absl::OkStatus();
}

absl::Status PassthroughDataSource::AddImageCallback(
    const ImageCallback& callback) {
  absl::MutexLock lock(&mutex_);
  image_callbacks_.push_back(callback);
  return absl::OkStatus();
}

absl::Status PassthroughDataSource::AddOnCompletedCallback(
    const OnCompletedCallback& callback) {
  // Never completes.
  return absl::OkStatus();
}

void PassthroughDataSource::OnImage(
    const std::shared_ptr<const util::Image>& image) {
  absl::MutexLock lock(&mutex_);
  for (const auto& callback : image_callbacks_) {
    callback(image);
  }
}

absl::Status PassthroughDataSource::SetGlTextureOutput(
    const GlTextureCallback& gl_texture_callback) {
  return absl::UnimplementedError("SetGlTextureOutput not implemented");
}

absl::Status PassthroughDataSource::GlAttachTexture(int32_t texture_id) {
  return absl::UnimplementedError("GlAttachTexture not implemented");
}
absl::Status PassthroughDataSource::GlUpdateTexture() {
  return absl::UnimplementedError("GlUpdateTexture not implemented");
}

}  // namespace guideline::data
