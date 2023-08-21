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

#include "project_guideline/android/arcore/arcore_motion_tracker.h"

#include <memory>
#include <optional>

#include "absl/functional/bind_front.h"
#include "absl/log/check.h"
#include "absl/synchronization/mutex.h"
#include "Eigen/Core"
#include "project_guideline/android/arcore/arcore_session.h"
#include "project_guideline/camera/camera_model.h"
#include "project_guideline/camera/cv_camera_model.h"
#include "project_guideline/motion/tracking_feature.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/transformation.h"

namespace guideline::arcore {

namespace {
using Eigen::Affine3d;
using Eigen::AngleAxisd;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using util::kDegreesToRadians;
using util::Transformation;

// Converts a pose from ARCore OpenGL coordinate frame (Y up, -Z forward) to
// the coordinate frame used by Guideline (Z up, Y forward).
util::Transformation ConvertArPose(const util::Transformation& ar_pose) {
  // ARCore uses OpenGL coordinate system.
  Transformation glWorld_T_glDisplayCamera = ar_pose;

  // Converts from Display -> Device -> Physical for rear camera in
  // reverse landscape (Surface.ROTATION_270).
  Affine3d affine_glDisplayCamera_Q_glPhysicalCamera = Affine3d::Identity();
  affine_glDisplayCamera_Q_glPhysicalCamera.prerotate(
      AngleAxisd(180 * kDegreesToRadians, Vector3d::UnitZ()));
  Transformation glDisplayCamera_Q_glPhysicalCamera =
      Transformation::FromMatrix(
          affine_glDisplayCamera_Q_glPhysicalCamera.matrix());

  Transformation glWorld_T_glPhysicalCamera(glWorld_T_glDisplayCamera *
                                            glDisplayCamera_Q_glPhysicalCamera);

  // GL to MT is a right-handed -90 around around X
  Affine3d affine_glWorld_Q_mtWorld = Affine3d::Identity();
  affine_glWorld_Q_mtWorld.prerotate(
      AngleAxisd(-90 * kDegreesToRadians, Vector3d::UnitX()));
  Transformation glWorld_T_mtWorld =
      Transformation::FromMatrix(affine_glWorld_Q_mtWorld.matrix());

  // Handle difference between Open GL and internal Motion Tracking
  // camera conventions.
  Affine3d affine_glCamera_Q_mtCamera = Affine3d::Identity();
  affine_glCamera_Q_mtCamera.prerotate(
      AngleAxisd(180 * kDegreesToRadians, Vector3d::UnitX()));
  Transformation glCamera_T_mtCamera =
      Transformation::FromMatrix(affine_glCamera_Q_mtCamera.matrix());

  Transformation mtWorld_T_mtPhysicalCamera =
      Transformation::FromMatrix(glWorld_T_mtWorld.Inverse().ToMatrix4x4() *
                                 glWorld_T_glPhysicalCamera.ToMatrix4x4() *
                                 glCamera_T_mtCamera.ToMatrix4x4());

  return mtWorld_T_mtPhysicalCamera;
}
}  // namespace

ArcoreMotionTracker::ArcoreMotionTracker(
    std::shared_ptr<ArcoreSession> arcore_session)
    : arcore_session_(arcore_session) {}

absl::Status ArcoreMotionTracker::Start() {
  absl::MutexLock lock(&mutex_);
  CHECK(frame_data_callback_key_ == std::nullopt) << "Already started";
  frame_data_callback_key_ = arcore_session_->AddFrameDataCallback(
      absl::bind_front(&ArcoreMotionTracker::OnFrameData, this));
  return absl::OkStatus();
}

absl::Status ArcoreMotionTracker::Stop() {
  absl::MutexLock lock(&mutex_);
  CHECK(frame_data_callback_key_ != std::nullopt) << "Already stopped";
  arcore_session_->RemoveFrameDataCallback(*frame_data_callback_key_);
  tracking_ = false;
  return absl::OkStatus();
}

void ArcoreMotionTracker::OnFrameData(const ArcoreFrameData& frame_data) {
  Eigen::Vector4f camera_pinhole_params = {
      frame_data.intrinsics_fx, frame_data.intrinsics_fy,
      frame_data.intrinsics_cx, frame_data.intrinsics_cy};

  if (camera_pinhole_params != last_camera_pinhole_params_) {
    last_camera_pinhole_params_ = camera_pinhole_params;

    // [k1, k2, k3]
    Eigen::Vector3f ar_distortion_coeffs =
        frame_data.instrinsics_distortion_coeffs;

    // [k1, k2, p1, p2, k3]
    Eigen::VectorXd cv_distortion_coeffs(5);
    cv_distortion_coeffs << ar_distortion_coeffs[0], ar_distortion_coeffs[1], 0,
        0, ar_distortion_coeffs[2];

    std::shared_ptr<camera::CameraModel> camera_model =
        std::make_unique<camera::CvCameraModel>(
            frame_data.intrinsics_image_width,
            frame_data.intrinsics_image_height,
            camera_pinhole_params.cast<double>(), cv_distortion_coeffs);
    UpdateCameraModel(camera_model);
  }

  if (tracking_.exchange(frame_data.tracking) != frame_data.tracking) {
    NotifyTracking(frame_data.tracking);
  }

  int64_t timestamp_us =
      absl::ToInt64Microseconds(absl::Nanoseconds(frame_data.timestamp_ns));

  if (frame_data.pose.has_value()) {
    util::Transformation pose = ConvertArPose(frame_data.pose.value());
    NotifyCameraMotion(timestamp_us, pose);

    if (!frame_data.point_cloud.empty()) {
      std::vector<motion::TrackingFeature> tracking_features;
      CHECK_EQ(frame_data.point_cloud.size(),
               frame_data.point_cloud_ids.size());

      // GL to MT is a right-handed -90 around around X
      Affine3d affine_glWorld_Q_mtWorld = Affine3d::Identity();
      affine_glWorld_Q_mtWorld.prerotate(
          AngleAxisd(-90 * kDegreesToRadians, Vector3d::UnitX()));
      Transformation glWorld_T_mtWorld =
          Transformation::FromMatrix(affine_glWorld_Q_mtWorld.matrix());
      util::Transformation mtWorld_T_camera = pose;

      for (int i = 0; i < frame_data.point_cloud.size(); ++i) {
        const auto& point = frame_data.point_cloud[i];
        const int32_t point_id = frame_data.point_cloud_ids[i];

        if (point.w() <= 0) {
          continue;
        }

        // The feature points from ARCore are in GL world coordinates.
        // Convert into MT camera frame coordinates.
        Eigen::Vector3d glWorld_T_feature = point.head<3>().cast<double>();

        Eigen::Vector3d mtWorld_T_feature =
            glWorld_T_mtWorld.Inverse().q() * glWorld_T_feature;

        Eigen::Vector3d mtCamera_T_feature =
            (mtWorld_T_camera.Inverse().q() *
             (mtWorld_T_feature - mtWorld_T_camera.p()));

        tracking_features.emplace_back(mtCamera_T_feature.cast<float>(),
                                       point.w(), point_id);
      }
      NotifyTrackingFeatures(timestamp_us, tracking_features);
    }
  }
}

}  // namespace guideline::arcore
