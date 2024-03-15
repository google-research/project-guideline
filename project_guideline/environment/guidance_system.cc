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

#include "project_guideline/environment/guidance_system.h"

#include <deque>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/time/time.h"
#include "Eigen/Core"
#include "project_guideline/depth/depth_align_ransac.h"
#include "project_guideline/depth/point_cloud_util.h"
#include "project_guideline/environment/obstacle.h"
#include "project_guideline/motion/tracking_feature.h"
#include "project_guideline/proto/control_signal.pb.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/image.h"
#include <google/protobuf/util/time_util.h>

namespace guideline::environment {

namespace {

using camera::CameraModel;
using depth::Point3D;
using ::Eigen::Vector2d;
using ::Eigen::Vector3d;
using logging::GuidelineLogger;
using motion::TrackingFeature;
using util::Transformation;

constexpr util::Axis3 kAxisGravity = util::Axis3::kZ;

// Finds the camera pose in the deque corresponding to the given timestamp,
// discarding any camera poses older than the timestamp, and assigns it to
// 'pose'.
// Returns true if a camera pose was found, or false otherwise.
bool DequeCameraPose(
    std::deque<std::pair<const int64_t, Transformation>>& deque,
    int64_t timestamp_us, Transformation& pose) {
  for (;; deque.pop_front()) {
    if (deque.empty() || deque.front().first > timestamp_us) {
      break;
    } else if (deque.front().first == timestamp_us) {
      pose = deque.front().second;
      return true;
    }
  }
  return false;
}

// Finds the tracking features in the queue that either:
// 1) exactly match the given frame timestamp
// 2) are the closest earlier tracking features
// 3) are the closest newer tracking features (if no earlier features exist)
// Older tracking features no longer needed are discarded.
// Returns true if tracking features were found, or false otherwise.
bool DequeTrackingFeatures(
    std::deque<std::pair<const int64_t, PendingFeatures>> deque,
    int64_t timestamp_us, PendingFeatures& features) {
  if (deque.empty()) {
    return false;
  }

  if (deque.front().first >= timestamp_us) {
    // The first element is the nearest to the timestamp.
    features = deque.front().second;
    return true;
  } else {
    while (deque.size() > 1) {
      // Keep discarding until we reach the closest timestamp.
      if (deque.at(1).first <= timestamp_us) {
        deque.pop_front();
      } else {
        break;
      }
    }
    features = deque.front().second;
    return true;
  }
}

void LogPointCloud(const int64_t timestamp_us, Environment& environment,
                   GuidelineLogger& logger) {
  if (logger.GetLogIntermediate()) {
    absl::StatusOr<std::vector<Point3D>> point_cloud =
        environment.point_cloud_->GetPointCloud();
    if (!point_cloud.ok()) {
      return;
    }

    GuidelineLogEvent event;
    event.set_frame_timestamp_us(timestamp_us);
    GuidelineLogEvent_PointCloud* point_cloud_log = event.mutable_point_cloud();
    for (const Point3D& point : *point_cloud) {
      point_cloud_log->add_packed_world_point(point.coordinate(0));
      point_cloud_log->add_packed_world_point(point.coordinate(1));
      point_cloud_log->add_packed_world_point(point.coordinate(2));
      point_cloud_log->add_packed_world_point(point.confidence);
    }
    logger.LogEvent(std::move(event));
  }
}

}  // namespace

absl::StatusOr<std::unique_ptr<GuidanceSystem>> GuidanceSystem::Create(
    const GuidanceSystemOptions& options,
    std::shared_ptr<GuidelineLogger> logger,
    std::unique_ptr<Environment> environment,
    std::unique_ptr<ControlSystem> control_system) {
  return absl::WrapUnique(new GuidanceSystem(
      options, logger, std::move(environment), std::move(control_system)));
}

GuidanceSystem::GuidanceSystem(const GuidanceSystemOptions& options,
                               std::shared_ptr<GuidelineLogger> logger,
                               std::unique_ptr<Environment> environment,
                               std::unique_ptr<ControlSystem> control_system)
    : options_(options),
      logger_(logger),
      environment_(std::move(environment)),
      control_system_(std::move(control_system)) {}

absl::Status GuidanceSystem::Stop() {
  ResetAndSendStopSignal(StopReason::STOP_REASON_APP_INITIATED);

  return absl::OkStatus();
}

void GuidanceSystem::OnCameraPose(const int64_t timestamp_us,
                                  const Transformation& world_t_camera,
                                  std::shared_ptr<CameraModel> camera_model) {
  {
    absl::MutexLock lock(&camera_model_mutex_);
    camera_model_ = camera_model;
  }
  {
    absl::MutexLock lock(&pending_camera_poses_mutex_);
    pending_detection_poses_.emplace_back(timestamp_us, world_t_camera);
    pending_feature_poses_.emplace_back(timestamp_us, world_t_camera);
    // Human stateful representation is updated with latest camera pose.
    environment_->human_->UpdatePositionAndDirection(world_t_camera,
                                                     timestamp_us);
  }

  // Updates occupancy map only when obstacle detection is enabled.
  std::vector<std::pair<Vector2d, int>> occupancy_map;
  if (options_.enable_obstacle_detection()) {
    absl::StatusOr<std::vector<Point3D>> point_cloud =
        environment_->point_cloud_->GetPointCloud();
    if (!point_cloud.ok()) {
      LOG(WARNING) << "Failed to get point cloud at " << timestamp_us;
      return;
    }

    absl::StatusOr<Transformation> human_position_direction =
        environment_->human_->CurrentPositionAndDirection();
    if (!human_position_direction.ok()) {
      LOG(WARNING) << "Failed to get human position and direction at "
                   << timestamp_us;
      return;
    }
    environment_->occupancy_map_->UpdateOccupancyMap(*point_cloud,
                                                     *human_position_direction);
    occupancy_map = environment_->occupancy_map_->GetOccupancyMap();
  }

  std::vector<Vector3d> guideline_points;
  if (!options_.obstacle_only_mode()) {
    absl::StatusOr<std::vector<Eigen::Vector3d>> guideline =
        environment_->guideline_aggregator_->GetGuideline();
    if (!guideline.ok()) {
      LOG(WARNING) << "Failed to get guideline at " << timestamp_us;
      return;
    }
    guideline_points = std::move(*guideline);
  }

  auto velocity = environment_->human_->VelocityVector();

  std::vector<Obstacle> obstacles;
  if (!occupancy_map.empty()) {
    GuidelineLogEvent occupancy_event;
    occupancy_event.set_frame_timestamp_us(timestamp_us);
    GuidelineLogEvent_OccupancyMap* occupancy_map_log =
        occupancy_event.mutable_occupancy_map();

    for (const auto& grid : occupancy_map) {
      // TODO(b/262876436): This currently treats each grid as a separate
      // obstacle. There should be some logic group grids together into objects.
      if (grid.second > 0) {
        Transformation grid_position({grid.first.x(), grid.first.y(), 0});
        auto obstacle = obstacles.emplace_back();
        obstacle.UpdatePositionAndDirection(grid_position, timestamp_us);
      }

      occupancy_map_log->add_packed_occupancy(grid.first.x());
      occupancy_map_log->add_packed_occupancy(grid.first.y());
      occupancy_map_log->add_packed_occupancy(static_cast<float>(grid.second));
    }

    logger_->LogEvent(std::move(occupancy_event));
  }

  auto control_signal = control_system_->GenerateControlSignal(
      world_t_camera, velocity.ok() ? velocity.value() : Vector3d(0, 0, 0),
      guideline_points, obstacles, kAxisGravity);
  {
    absl::MutexLock lock(&control_signal_callbacks_mutex_);
    for (const auto& callback : control_signal_callbacks_) {
      callback(control_signal);
    }
  }

  // TODO(b/221399299): If `STOP` signal then go back to an initial state to
  // handle runner restart (e.g. reorienting to the line).

  GuidelineLogEvent control_event;
  control_event.set_frame_timestamp_us(timestamp_us);
  auto control_signal_log = control_event.mutable_control_signal();
  control_signal_log->set_lateral_movement_meters(
      control_signal.lateral_movement_meters);
  control_signal_log->set_rotation_movement_degrees(
      control_signal.rotation_movement_degrees);
  control_signal_log->set_stop(control_signal.stop);
  control_signal_log->set_turn_angle_degrees(control_signal.turn_angle_degrees);
  if (control_signal.turn_point.has_value()) {
    control_signal_log->mutable_turn_point()->set_x(
        control_signal.turn_point.value().x());
    control_signal_log->mutable_turn_point()->set_y(
        control_signal.turn_point.value().y());
    control_signal_log->mutable_turn_point()->set_z(
        control_signal.turn_point.value().z());
  }
  control_signal_log->set_obstacle_ahead(control_signal.obstacle_ahead);
  logger_->LogEvent(std::move(control_event));
}

void GuidanceSystem::OnTrackingStateChanged(const bool is_tracking) {
  // Handle tracking stopped case by clearing the environment and issuing a
  // stop signal. When we lose tracking the environment is no longer valid.
  // Nothing need to be done specifically when tracking resumes since
  // OnCameraPose will start being invoked again with from a new reference
  // point.
  if (!is_tracking) {
    ResetAndSendStopSignal(StopReason::STOP_REASON_TRACKING_STATE_CHANGED);
  }
}

void GuidanceSystem::OnTrackingFeatures(
    const int64_t timestamp_us, const std::vector<TrackingFeature>& features) {
  Transformation world_t_camera;
  {
    absl::MutexLock lock(&pending_camera_poses_mutex_);
    if (!DequeCameraPose(pending_feature_poses_, timestamp_us,
                         world_t_camera)) {
      LOG(WARNING) << "Failed to find camera pose for tracking features "
                   << timestamp_us;
      return;
    }
    pending_tracking_features_.emplace_back(
        timestamp_us, PendingFeatures{timestamp_us, world_t_camera, features});
  }
}

void GuidanceSystem::UpdateStatefulRepresentation(
    const Transformation& world_t_camera,
    absl::Span<const util::HitResult> ordered_hit_results,
    int64_t timestamp_us) {
  // Human stateful representation is updated in `OnCameraPose`.
  if (!options_.obstacle_only_mode()) {
    environment_->guideline_aggregator_->AggregateKeypoints(
        ordered_hit_results, timestamp_us, kAxisGravity);
  }
}

void GuidanceSystem::OnDetection(
    const int64_t timestamp_us, const std::vector<Eigen::Vector3f>& keypoints,
    std::shared_ptr<const util::ConfidenceMask> guideline_mask,
    std::shared_ptr<const util::DepthImage> depth_map) {
  Transformation world_t_camera;
  PendingFeatures features;
  {
    absl::MutexLock lock(&pending_camera_poses_mutex_);
    if (!DequeCameraPose(pending_detection_poses_, timestamp_us,
                         world_t_camera)) {
      LOG(WARNING) << "Failed to find corresponding camera pose "
                   << timestamp_us;
      return;
    }
    if (!DequeTrackingFeatures(pending_tracking_features_, timestamp_us,
                               features)) {
      LOG(WARNING) << "Failed to find corresponding tracking features "
                   << timestamp_us;
    }
  }

  std::shared_ptr<CameraModel> camera_model = GetCameraModel();

  if (options_.enable_obstacle_detection() && !depth_map->data().empty()) {
    ProcessDepthMap(timestamp_us, world_t_camera, *depth_map, features);
  }

  GuidelineLogEvent event;
  event.set_frame_timestamp_us(timestamp_us);
  auto world_update = event.mutable_world_update();

  Transformation world_T_plane = util::CreateSyntheticGroundPlane(
      world_t_camera, options_.camera_height_meters());

  std::vector<util::HitResult> ordered_hit_results;
  ordered_hit_results.reserve(keypoints.size());

  for (const auto& keypoint : keypoints) {
    Vector2d image_point =
        Vector2d(keypoint.x() * camera_model->image_width(),
                 keypoint.y() * camera_model->image_height());
    auto status_or_hit = util::HitTestPlane(image_point, world_T_plane,
                                            world_t_camera, *camera_model);
    if (!status_or_hit.ok()) {
      continue;
    }
    auto hit_result = status_or_hit.value();
    ordered_hit_results.push_back(hit_result);

    auto hit_log = world_update->add_hit();
    auto keypoint_log = hit_log->mutable_keypoint();
    keypoint_log->set_x(keypoint.x());
    keypoint_log->set_y(keypoint.y());
    keypoint_log->set_score(keypoint.z());

    hit_log->set_hit_distance(hit_result.hit_distance);

    auto translation = hit_log->mutable_hit_translation();
    translation->set_x(hit_result.hit_pose.p().x());
    translation->set_y(hit_result.hit_pose.p().y());
    translation->set_z(hit_result.hit_pose.p().z());
  }

  UpdateStatefulRepresentation(world_t_camera, ordered_hit_results,
                               timestamp_us);
  logger_->LogEvent(std::move(event));

  if (options_.has_eager_stop_threshold() && !options_.obstacle_only_mode()) {
    absl::Duration eager_stop_threshold =
        absl::Microseconds(google::protobuf::util::TimeUtil::DurationToMicroseconds(
            options_.eager_stop_threshold()));

    // Send stop signal if no keypoint has been detected for the past 1 second
    if (!keypoints.empty()) {
      first_empty_keypoints_timestamp_us_ = 0;
    } else if (first_empty_keypoints_timestamp_us_ == 0) {
      first_empty_keypoints_timestamp_us_ = timestamp_us;
    } else if (absl::Microseconds(timestamp_us -
                                  first_empty_keypoints_timestamp_us_) >=
               eager_stop_threshold) {
      first_empty_keypoints_timestamp_us_ = 0;
      ResetAndSendStopSignal(StopReason::STOP_REASON_NO_KEY_POINT);
    }
  }
}

void GuidanceSystem::ProcessDepthMap(int64_t timestamp_us,
                                     const Transformation& world_t_camera,
                                     const util::DepthImage& depth_map,
                                     const PendingFeatures& features) {
  std::shared_ptr<CameraModel> camera_model = GetCameraModel();
  absl::StatusOr<depth::DepthAlignParams> depth_align_params;
  if (!options_.align_depth_with_features()) {
    environment_->point_cloud_->UpdatePointCloud(depth_map, world_t_camera,
                                                 *camera_model);
    LogPointCloud(timestamp_us, *environment_, *logger_);
    return;
  }

  std::vector<TrackingFeature> tracking_features;
  if (features.timestamp_us == timestamp_us) {
    tracking_features = features.features;
  } else {
    // The tracking features are from a different frame, and expressed in
    // coordinates relative to that frame's camera pose. Remap the features to
    // be expressed in the current camera frame of reference.
    Transformation current_camera_t_features_camera =
        world_t_camera.Inverse() * features.world_t_camera;
    tracking_features.reserve(features.features.size());
    for (const TrackingFeature& feature0 : features.features) {
      const Eigen::Vector3d current_camera_t_feature =
          current_camera_t_features_camera *
          feature0.camera_t_feature.cast<double>();
      tracking_features.emplace_back(current_camera_t_feature.cast<float>(),
                                     feature0.confidence, feature0.feature_id);
    }
  }

  // Run RANSAC solver to match the depth of tracked VIO features with
  // corresponding points on the depth map. Since ML depth is somewhat relative,
  // this greatly improves the results and aligns the point cloud to the world.
  depth_align_params =
      depth::DepthAlignRansac(depth_map, tracking_features, *camera_model);
  if (!depth_align_params.ok()) {
    LOG(WARNING) << "Failed to align depth: " << depth_align_params.status();
    return;
  }

  std::vector<float> depth_aligned;
  auto depth_aligned_mat =
      std::make_unique<cv::Mat>(depth_map.height(), depth_map.width(), CV_32F);
  auto aligned_iter = depth_aligned_mat->begin<float>();
  for (auto iter = depth_map.data().begin<float>();
       iter != depth_map.data().end<float>(); ++iter, ++aligned_iter) {
    float depth = *iter;
    const float inv_depth = depth < std::numeric_limits<float>::epsilon()
                                ? std::numeric_limits<float>::max()
                                : 1.0 / depth;
    const float pred_inv_depth =
        inv_depth * depth_align_params->scale + depth_align_params->shift;
    const float pred_depth =
        pred_inv_depth < std::numeric_limits<float>::epsilon()
            ? std::numeric_limits<float>::max()
            : 1.0 / pred_inv_depth;
    *aligned_iter = pred_depth;
  }

  const util::DepthImage depth_map_aligned(
      depth_map.width(), depth_map.height(), std::move(depth_aligned_mat));
  environment_->point_cloud_->UpdatePointCloud(depth_map_aligned,
                                               world_t_camera, *camera_model);
  LogPointCloud(timestamp_us, *environment_, *logger_);
}

void GuidanceSystem::ResetAndSendStopSignal(StopReason reason) {
  {
    absl::MutexLock lock(&pending_camera_poses_mutex_);
    pending_detection_poses_.clear();
    pending_feature_poses_.clear();
    pending_tracking_features_.clear();
  }
  CHECK_OK(environment_->ClearAll());
  CHECK_OK(control_system_->ClearEntries(std::nullopt));
  // Generate a stop signal and notify callbacks.
  ControlSignal stop_signal;
  stop_signal.stop = true;
  stop_signal.stop_reason = reason;

  std::vector<ControlSignalCallback> callbacks;
  {
    absl::MutexLock lock(&control_signal_callbacks_mutex_);
    callbacks = control_signal_callbacks_;
  }
  for (const auto& callback : callbacks) {
    callback(stop_signal);
  }
}

void GuidanceSystem::OnArDepth(const int64_t timestamp_us,
                               const util::DepthImageU16& depth_image,
                               const util::ConfidenceImageU8& confidence) {
  if (!options_.enable_obstacle_detection()) {
    return;
  }
  Transformation world_t_camera;
  {
    // OnArDepth is called after OnCameraPose, so the last camera pose will
    // still be in the queue even if OnDetection gets called first.
    absl::MutexLock lock(&pending_camera_poses_mutex_);
    if (!DequeCameraPose(pending_detection_poses_, timestamp_us,
                         world_t_camera)) {
      LOG(WARNING) << "Failed to find corresponding camera pose for ArDepth "
                   << timestamp_us;
      return;
    }
  }
  environment_->point_cloud_->UpdatePointCloud(
      depth_image, confidence, world_t_camera, *GetCameraModel());
  LogPointCloud(timestamp_us, *environment_, *logger_);
}

void GuidanceSystem::AddControlSignalCallback(
    const ControlSignalCallback& callback) {
  absl::MutexLock lock(&control_signal_callbacks_mutex_);
  control_signal_callbacks_.push_back(callback);
}

absl::StatusOr<std::vector<Eigen::Vector3d>> GuidanceSystem::GetGuideline() {
  return environment_->guideline_aggregator_->GetGuideline();
}

absl::StatusOr<std::vector<Eigen::Vector2d>> GuidanceSystem::GetObstacles() {
  std::vector<Eigen::Vector2d> obstacles;
  auto occupancy_map = environment_->occupancy_map_->GetOccupancyMap();
  for (const auto& grid : occupancy_map) {
    if (grid.second > 0) {
      obstacles.push_back(grid.first);
    }
  }
  return obstacles;
}

absl::StatusOr<Transformation> GuidanceSystem::CurrentPositionAndDirection() {
  return environment_->human_->CurrentPositionAndDirection();
}

std::shared_ptr<camera::CameraModel> GuidanceSystem::GetCameraModel() {
  absl::MutexLock lock(&camera_model_mutex_);
  std::shared_ptr<camera::CameraModel> camera_model = camera_model_;
  CHECK(camera_model) << "CameraModel not available";
  return camera_model;
}

}  // namespace guideline::environment
