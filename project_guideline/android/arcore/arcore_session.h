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

#ifndef PROJECT_GUIDELINE_ANDROID_ARCORE_ARCORE_SESSION_H_
#define PROJECT_GUIDELINE_ANDROID_ARCORE_ARCORE_SESSION_H_

#include <jni.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <thread>  // NOLINT(build/c++11)
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "arcore_c_api.h"
#include "Eigen/Core"
#include "project_guideline/util/image.h"
#include "project_guideline/util/transformation.h"

namespace guideline::arcore {

// Per-frame data that is obtained from ARCore.
struct ArcoreFrameData {
  ArcoreFrameData() = default;
  ArcoreFrameData(const ArcoreFrameData&) = delete;

  // The timestamp of the frame, which the timestamp of the image captured
  // from the camera.
  int64_t timestamp_ns = 0;

  // Whether ARCore is currently in tracking state (camera pose available).
  bool tracking = false;

  // Camera pose corresponding to this frame. Only present when tracking = true.
  std::optional<util::Transformation> pose = std::nullopt;

  std::shared_ptr<util::Image> image = nullptr;

  int intrinsics_image_width = 0;
  int intrinsics_image_height = 0;
  float intrinsics_fx = 0;
  float intrinsics_fy = 0;
  float intrinsics_cx = 0;
  float intrinsics_cy = 0;
  // Distortion coefficients [k1, k2, k3] (sometimes also referred to as
  // [k0, k1, k2]).
  Eigen::Vector3f instrinsics_distortion_coeffs;

  // Point cloud of 3d points being tracked by ARCore. Each point has format:
  // [x, w, z, confidence] where confidence is a value between [0, 1].
  std::vector<Eigen::Vector4f> point_cloud;

  // Unique ids corresponding to each point in point_cloud.
  std::vector<int32_t> point_cloud_ids;
};

// Wrapper that manages an ArSession.
class ArcoreSession {
 public:
  using FrameDataCallback =
      std::function<void(const ArcoreFrameData& frame_data)>;

  using OnDrawTextureUpdateCallback = std::function<void(bool texture_valid)>;

  ArcoreSession(Eigen::Vector3f distortion_coeffs = Eigen::Vector3f::Zero())
      : distortion_coeffs_(distortion_coeffs) {}
  ~ArcoreSession();

  // Invoked when the host activity is resumed.
  absl::Status OnResume(JNIEnv* env, jobject context);

  // Invoked when the host activity is paused.
  absl::Status OnPause();

  // Set the OpenGL texture id to use for the camera image.
  // The texture must be bound to the GL_TEXTURE_EXTERNAL_OES target for use.
  // Shaders accessing this texture must use a samplerExternalOES sampler.
  void SetGlCameraTexture(uint32_t texture_id);

  // Invoked from GLSurfaceView.Renderer.onDrawFrame() which updates the
  // ARCore frame and causes any FrameDataCallbacks to be called with the
  // updated data.
  void OnDrawFrame();

  // Invoked from GLSurfaceView.Renderer.onSurfaceChanged() when the GL
  // viewport has changed size.
  void SetViewport(int width, int height);

  // Adds a callback that will be invoked with updated frame data from ARCore.
  // The callback will be called from a worker thread.
  // Returns a key that can be used to remove the callback with
  // RemoveFrameDataCallback().
  int AddFrameDataCallback(const FrameDataCallback& callback);

  // Removes a callback previously added with AddFrameDataCallback.
  void RemoveFrameDataCallback(int key);

  void SetOnDrawTextureUpdateCallback(
      const OnDrawTextureUpdateCallback& callback);

 private:
  absl::Status CreateSession(JNIEnv* env, jobject context)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void DestroySession() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  absl::StatusOr<std::unique_ptr<ArcoreFrameData>> GetFrameData(
      int64_t timestamp_ns) ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  void RunNotify();

  const bool enable_depth_ = false;
  Eigen::Vector3f distortion_coeffs_;

  absl::Mutex mutex_;
  ArSession* session_ ABSL_GUARDED_BY(mutex_) = nullptr;
  ArFrame* frame_ ABSL_GUARDED_BY(mutex_) = nullptr;
  int64_t last_frame_timestamp_ns_ ABSL_GUARDED_BY(mutex_) = 0;
  uint32_t camera_texture_id_ ABSL_GUARDED_BY(mutex_) = 0;
  OnDrawTextureUpdateCallback ABSL_GUARDED_BY(mutex_)
      on_draw_texture_update_callback_;

  std::unique_ptr<std::thread> notify_thread_ ABSL_GUARDED_BY(mutex_) = nullptr;
  std::atomic<bool> resumed_ = false;
  std::atomic<bool> install_requested_ = false;

  absl::Mutex notify_mutex_;
  std::map<int, FrameDataCallback> frame_data_callbacks_
      ABSL_GUARDED_BY(notify_mutex_);
  int next_callback_key_ ABSL_GUARDED_BY(notify_mutex_) = 0;
  std::unique_ptr<ArcoreFrameData> notify_frame_data_
      ABSL_GUARDED_BY(notify_mutex_) = nullptr;
};

}  // namespace guideline::arcore

#endif  // PROJECT_GUIDELINE_ANDROID_ARCORE_ARCORE_SESSION_H_
