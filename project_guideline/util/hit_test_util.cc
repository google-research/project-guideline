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

#include "project_guideline/util/hit_test_util.h"

#include "absl/status/status.h"

namespace guideline::util {

namespace {
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
}  // namespace

Transformation CreateSyntheticGroundPlane(Transformation& camera_pose,
                                          float camera_height_meters) {
  return {Eigen::Vector3d(camera_pose.p().x(), camera_pose.p().y(),
                          camera_pose.p().z() - camera_height_meters)};
}

absl::StatusOr<HitResult> HitTestPlane(
    const Vector2d& pixel, const Transformation& world_T_plane,
    const Transformation& world_T_camera,
    const camera::CameraModel& camera_model) {
  const Transformation camera_T_plane =
      world_T_camera.Inverse() * world_T_plane;

  Vector3d camera_ray_direction;
  if (!camera_model.PixelToRay(pixel, camera_ray_direction)) {
    return absl::InternalError("Unable to project pixel to ray.");
  }

  const Vector3d camera_plane_normal =
      camera_T_plane.q().toRotationMatrix().col(2);
  const double ray_and_normal_dot_product =
      camera_ray_direction.dot(camera_plane_normal);
  if (std::abs(ray_and_normal_dot_product) <
      std::numeric_limits<double>::epsilon()) {
    return absl::InternalError("Ray is parallel to plane.");
  }

  const Vector3d camera_plane_point_from_pixel =
      camera_T_plane.p().dot(camera_plane_normal) / ray_and_normal_dot_product *
      camera_ray_direction;

  if (camera_ray_direction.dot(camera_plane_point_from_pixel) < 0.f) {
    return absl::InternalError("Point does not intersect plane.");
  }

  Transformation world_T_plane_hit = Transformation(
      world_T_plane.q(), world_T_camera * camera_plane_point_from_pixel);
  float distance_to_hit = (world_T_plane_hit.p() - world_T_camera.p()).norm();

  return HitResult(
      Transformation(world_T_camera * camera_plane_point_from_pixel),
      distance_to_hit);
}

}  // namespace guideline::util
