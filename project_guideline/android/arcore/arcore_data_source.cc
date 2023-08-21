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

#include "project_guideline/android/arcore/arcore_data_source.h"

#include <optional>

#include "absl/functional/bind_front.h"
#include "absl/log/check.h"
#include "absl/synchronization/mutex.h"

namespace guideline::arcore {

ArcoreDataSource::ArcoreDataSource(std::shared_ptr<ArcoreSession> session)
    : session_(session) {}

absl::Status ArcoreDataSource::Start() {
  absl::MutexLock lock(&mutex_);
  CHECK(session_callback_key_ == std::nullopt) << "Data source already started";
  session_callback_key_ = session_->AddFrameDataCallback(
      absl::bind_front(&ArcoreDataSource::OnFrameData, this));
  return absl::OkStatus();
}
absl::Status ArcoreDataSource::Stop() {
  absl::MutexLock lock(&mutex_);
  CHECK(session_callback_key_) << "Data source already stopped";
  session_->RemoveFrameDataCallback(*session_callback_key_);
  return absl::OkStatus();
}

absl::Status ArcoreDataSource::AddImageCallback(const ImageCallback& callback) {
  absl::MutexLock lock(&mutex_);
  image_callbacks_.push_back(callback);
  return absl::OkStatus();
}

absl::Status ArcoreDataSource::AddOnCompletedCallback(
    const OnCompletedCallback& callback) {
  // Never completes.
  return absl::OkStatus();
}

absl::Status ArcoreDataSource::SetGlTextureOutput(
    const GlTextureCallback& gl_texture_callback) {
  session_->SetOnDrawTextureUpdateCallback(
      [gl_texture_callback](bool texture_valid) {
        if (texture_valid) {
          gl_texture_callback();
        }
      });
  return absl::OkStatus();
}

absl::Status ArcoreDataSource::GlAttachTexture(int32_t texture_id) {
  session_->SetGlCameraTexture(texture_id);
  return absl::OkStatus();
}

absl::Status ArcoreDataSource::GlUpdateTexture() {
  // Texture update happens through ArcoreSession.OnDrawFrame().
  return absl::OkStatus();
}

void ArcoreDataSource::OnFrameData(const ArcoreFrameData& frame_data) {
  absl::MutexLock lock(&mutex_);
  for (const auto& callback : image_callbacks_) {
    callback(frame_data.image);
  }
}

}  // namespace guideline::arcore
