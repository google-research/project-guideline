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

#include "project_guideline/android/arcore/arcore_session.h"

#include <jni.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "absl/functional/bind_front.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/synchronization/mutex.h"
#include "arcore_c_api.h"
#include "project_guideline/util/image_yuv_conversion.h"
#include "project_guideline/util/status.h"
#include "project_guideline/util/transformation.h"

namespace guideline::arcore {

namespace {

// Holds a pointer to an ARCore API object which has a deleter function called
// when this object goes out of scope.
template <typename T>
struct ArScopedPtr {
  ArScopedPtr(void (*deleter)(T*)) : deleter_(deleter) {}
  ~ArScopedPtr() {
    if (ptr) {
      deleter_(ptr);
      ptr = nullptr;
    }
  }

  T* operator*() { return ptr; }

  T& operator->() { return *ptr; }

  T* ptr = nullptr;

 private:
  void (*deleter_)(T*);
};

}  // namespace

ArcoreSession::~ArcoreSession() {
  absl::MutexLock lock(&mutex_);
  DestroySession();
}

absl::Status ArcoreSession::OnResume(JNIEnv* env, jobject context) {
  absl::MutexLock lock(&mutex_);

  if (session_ == nullptr) {
    GL_RETURN_IF_ERROR(CreateSession(env, context));
  }

  ArStatus status = ArSession_resume(session_);
  if (status != AR_SUCCESS) {
    return absl::InternalError(
        absl::StrFormat("ArSession_resume failed: %d", status));
  }

  resumed_ = true;

  CHECK(notify_thread_ == nullptr);
  notify_thread_ = std::make_unique<std::thread>(
      absl::bind_front(&ArcoreSession::RunNotify, this));

  return absl::OkStatus();
}

absl::Status ArcoreSession::OnPause() {
  absl::MutexLock lock(&mutex_);
  if (session_) {
    ArStatus ar_status = ArSession_pause(session_);
    if (ar_status != AR_SUCCESS) {
      return absl::InternalError(
          absl::StrFormat("ArSession_pause failed: %d", ar_status));
    }
    if (on_draw_texture_update_callback_) {
      on_draw_texture_update_callback_(/*texture_valid=*/false);
    }
  }

  resumed_ = false;

  if (notify_thread_) {
    // Wake up the notify thread to process paused condition.
    notify_mutex_.Lock();
    notify_mutex_.Unlock();

    notify_thread_->join();
    notify_thread_ = nullptr;
  }

  return absl::OkStatus();
}

int ArcoreSession::AddFrameDataCallback(const FrameDataCallback& callback) {
  absl::MutexLock lock(&notify_mutex_);
  int key = ++next_callback_key_;
  frame_data_callbacks_.insert_or_assign(key, callback);
  return key;
}

void ArcoreSession::RemoveFrameDataCallback(int key) {
  absl::MutexLock lock(&notify_mutex_);
  frame_data_callbacks_.erase(key);
}

void ArcoreSession::SetOnDrawTextureUpdateCallback(
    const OnDrawTextureUpdateCallback& callback) {
  absl::MutexLock lock(&mutex_);
  on_draw_texture_update_callback_ = callback;
}

void ArcoreSession::SetGlCameraTexture(uint32_t texture_id) {
  absl::MutexLock lock(&mutex_);
  camera_texture_id_ = texture_id;

  if (session_) {
    ArSession_setCameraTextureName(session_, camera_texture_id_);
  }
}

void ArcoreSession::OnDrawFrame() {
  absl::MutexLock lock(&mutex_);
  CHECK(session_);
  CHECK(frame_);

  // Update the ArSession which will block some amount of time on a new camera
  // frame and update the camera pose and other system state.
  if (ArSession_update(session_, frame_) != AR_SUCCESS) {
    LOG(WARNING) << "ArSession_update failed";
    return;
  }

  int64_t timestamp_ns;
  ArFrame_getTimestamp(session_, frame_, &timestamp_ns);
  if (last_frame_timestamp_ns_ == timestamp_ns) {
    // Skip if there is no new frame to process.
    return;
  }
  last_frame_timestamp_ns_ = timestamp_ns;

  if (on_draw_texture_update_callback_ && camera_texture_id_ != 0) {
    on_draw_texture_update_callback_(/*texture_valid=*/true);
  }

  auto frame_data = GetFrameData(timestamp_ns);
  if (!frame_data.ok()) {
    LOG(WARNING) << "Failed to get frame data: " << frame_data.status();
    return;
  }

  {
    absl::MutexLock notify_lock(&notify_mutex_);
    notify_frame_data_ = std::move(*frame_data);
  }
}

void ArcoreSession::SetViewport(int width, int height) {
  absl::MutexLock lock(&mutex_);
  CHECK(session_);
  // Hardcoded for reverse landscape orientation.
  constexpr int kRotation270 = 3;
  ArSession_setDisplayGeometry(session_, kRotation270, width, height);
}

absl::StatusOr<std::unique_ptr<ArcoreFrameData>> ArcoreSession::GetFrameData(
    int64_t timestamp_ns) {
  auto frame_data = std::make_unique<ArcoreFrameData>();
  frame_data->timestamp_ns = timestamp_ns;

  ArScopedPtr<ArCamera> camera(&ArCamera_release);
  ArFrame_acquireCamera(session_, frame_, &camera.ptr);

  // Determine the tracking state (whether a camera pose is available).
  ArTrackingState tracking_state;
  ArCamera_getTrackingState(session_, *camera, &tracking_state);
  frame_data->tracking = (tracking_state == AR_TRACKING_STATE_TRACKING);

  if (tracking_state == AR_TRACKING_STATE_STOPPED) {
    ArTrackingFailureReason tracking_failure_reason;
    ArCamera_getTrackingFailureReason(session_, *camera,
                                      &tracking_failure_reason);
    LOG(WARNING) << "AR tracking stopped. reason: " << tracking_failure_reason;
  }

  // Get the camera intrinsic parameters.
  ArScopedPtr<ArCameraIntrinsics> camera_intrinsics(
      &ArCameraIntrinsics_destroy);
  ArCameraIntrinsics_create(session_, &camera_intrinsics.ptr);
  ArCamera_getImageIntrinsics(session_, *camera, *camera_intrinsics);

  ArCameraIntrinsics_getFocalLength(session_, *camera_intrinsics,
                                    &frame_data->intrinsics_fx,
                                    &frame_data->intrinsics_fy);
  ArCameraIntrinsics_getPrincipalPoint(session_, *camera_intrinsics,
                                       &frame_data->intrinsics_cx,
                                       &frame_data->intrinsics_cy);
  ArCameraIntrinsics_getImageDimensions(session_, *camera_intrinsics,
                                        &frame_data->intrinsics_image_width,
                                        &frame_data->intrinsics_image_height);
  frame_data->instrinsics_distortion_coeffs = distortion_coeffs_;

  // Get camera pose.
  util::Transformation pose_transformation;
  if (frame_data->tracking) {
    ArScopedPtr<ArPose> pose(&ArPose_destroy);
    ArPose_create(session_, nullptr, &pose.ptr);
    ArCamera_getDisplayOrientedPose(session_, *camera, *pose);
    if (!pose.ptr) {
      return absl::InternalError("Failed to get camera pose");
    }

    float raw_pose[7];
    ArPose_getPoseRaw(session_, *pose, raw_pose);
    float qx = raw_pose[0];
    float qy = raw_pose[1];
    float qz = raw_pose[2];
    float qw = raw_pose[3];
    float tx = raw_pose[4];
    float ty = raw_pose[5];
    float tz = raw_pose[6];
    frame_data->pose = util::Transformation({qw, qx, qy, qz}, {tx, ty, tz});
  }

  // Get and convert the camera CPU image.
  ArScopedPtr<ArImage> image(&ArImage_release);
  if (ArFrame_acquireCameraImage(session_, frame_, &image.ptr) != AR_SUCCESS) {
    return absl::InternalError("Failed to get camera image");
  }

  ArImageFormat image_format;
  ArImage_getFormat(session_, *image, &image_format);
  CHECK_EQ(image_format, AR_IMAGE_FORMAT_YUV_420_888)
      << "Unexpected image format: " << image_format;

  int32_t image_width, image_height;
  ArImage_getWidth(session_, *image, &image_width);
  ArImage_getHeight(session_, *image, &image_height);

  CHECK(frame_data->intrinsics_image_width == image_width &&
        frame_data->intrinsics_image_height == image_height)
      << "Image dimensions do not match camera instrinsics";

  int32_t num_planes;
  ArImage_getNumberOfPlanes(session_, *image, &num_planes);

  constexpr size_t kYuvNumPlanes = 3;
  CHECK_EQ(num_planes, kYuvNumPlanes)
      << "Unexpected number of image planes: " << num_planes;

  std::vector<util::ImagePlane> planes(kYuvNumPlanes);
  for (int i = 0; i < kYuvNumPlanes; ++i) {
    ArImage_getPlanePixelStride(session_, *image, i, &planes[i].pixel_stride);
    ArImage_getPlaneRowStride(session_, *image, i, &planes[i].row_stride);
    ArImage_getPlaneData(session_, *image, i, &planes[i].data,
                         &planes[i].data_length);
  }

  auto gl_image =
      ConvertYuvToRgb(image_width, image_height, planes, timestamp_ns);
  if (!gl_image.ok()) {
    return absl::InternalError("Failed to convert image");
  }

  frame_data->image = std::make_shared<util::Image>(std::move(*gl_image));

  // Get the point cloud which is used for 3d tracking features.
  ArScopedPtr<ArPointCloud> point_cloud(&ArPointCloud_release);
  if (ArFrame_acquirePointCloud(session_, frame_, &point_cloud.ptr) ==
      AR_SUCCESS) {
    int32_t num_points = 0;
    ArPointCloud_getNumberOfPoints(session_, *point_cloud, &num_points);

    if (num_points > 0) {
      const float* xyz_confidence = nullptr;
      ArPointCloud_getData(session_, *point_cloud, &xyz_confidence);
      const int32_t* point_ids = nullptr;
      ArPointCloud_getPointIds(session_, *point_cloud, &point_ids);
      for (int32_t i = 0; i < num_points; ++i) {
        frame_data->point_cloud.emplace_back(
            xyz_confidence[0], xyz_confidence[1], xyz_confidence[2],
            xyz_confidence[3]);
        xyz_confidence += 4;
        frame_data->point_cloud_ids.emplace_back(point_ids[i]);
      }
    }
  }

  return frame_data;
}

void ArcoreSession::RunNotify() {
  while (resumed_.load()) {
    notify_mutex_.Lock();
    auto has_frame_data_or_stopped =
        [this]() ABSL_SHARED_LOCKS_REQUIRED(notify_mutex_) {
          return !resumed_.load() || notify_frame_data_ != nullptr;
        };
    notify_mutex_.Await(absl::Condition(&has_frame_data_or_stopped));
    std::unique_ptr<ArcoreFrameData> frame_data = nullptr;
    if (notify_frame_data_) {
      frame_data = absl::WrapUnique(notify_frame_data_.release());
      notify_frame_data_ = nullptr;
    }
    std::map<int, FrameDataCallback> callbacks = frame_data_callbacks_;

    // Release the lock before notifying the callbacks.
    notify_mutex_.Unlock();

    if (frame_data) {
      for (const auto& callback : callbacks) {
        callback.second(*frame_data);
      }
    }
  }

  absl::MutexLock notify_lock(&notify_mutex_);
  notify_frame_data_ = nullptr;
}

absl::Status ArcoreSession::CreateSession(JNIEnv* env, jobject context) {
  ArStatus ar_status = ArSession_create(env, context, &session_);
  if (ar_status != AR_SUCCESS) {
    return absl::InternalError(
        absl::StrFormat("ArSession_create failed: %d", ar_status));
  }

  ArScopedPtr<ArConfig> config(&ArConfig_destroy);
  ArConfig_create(session_, &config.ptr);
  ArConfig_setDepthMode(
      session_, *config,
      enable_depth_ ? AR_DEPTH_MODE_AUTOMATIC : AR_DEPTH_MODE_DISABLED);
  ArConfig_setUpdateMode(session_, *config, AR_UPDATE_MODE_BLOCKING);

  ArFrame_create(session_, &frame_);
  CHECK(frame_) << "Failed to create ArFrame";

  // Set Camera target fps.
  ArScopedPtr<ArCameraConfigFilter> camera_config_filter(
      &ArCameraConfigFilter_destroy);
  ArCameraConfigFilter_create(session_, &camera_config_filter.ptr);
  ArCameraConfigFilter_setTargetFps(session_, *camera_config_filter,
                                    AR_CAMERA_CONFIG_TARGET_FPS_30);
  ArCameraConfigFilter_setFacingDirection(
      session_, *camera_config_filter, AR_CAMERA_CONFIG_FACING_DIRECTION_BACK);
  ArScopedPtr<ArCameraConfigList> camera_config_list(
      &ArCameraConfigList_destroy);
  ArCameraConfigList_create(session_, &camera_config_list.ptr);
  ArSession_getSupportedCameraConfigsWithFilter(session_, *camera_config_filter,
                                                *camera_config_list);
  ArScopedPtr<ArCameraConfig> camera_config(&ArCameraConfig_destroy);
  ArCameraConfig_create(session_, &camera_config.ptr);

  int32_t list_size = 0;
  ArCameraConfigList_getSize(session_, *camera_config_list, &list_size);
  if (list_size == 0) {
    return absl::InternalError("Failed to find a supported camera");
  }
  ArCameraConfigList_getItem(session_, *camera_config_list, 0, *camera_config);
  if (ArSession_setCameraConfig(session_, *camera_config) != AR_SUCCESS) {
    return absl::InternalError("ArSession_setCameraConfig failed");
  }

  if (ArSession_configure(session_, *config) != AR_SUCCESS) {
    return absl::InternalError("ArSession_configure failed");
  }

  if (camera_texture_id_) {
    ArSession_setCameraTextureName(session_, camera_texture_id_);
  }

  return absl::OkStatus();
}

void ArcoreSession::DestroySession() {
  if (notify_thread_) {
    notify_thread_->join();
    notify_thread_ = nullptr;
  }
  if (session_) {
    ArSession_destroy(session_);
    session_ = nullptr;
  }
  if (frame_) {
    ArFrame_destroy(frame_);
    frame_ = nullptr;
  }
}

}  // namespace guideline::arcore
