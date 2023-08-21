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

// Utilities for performing 'hit tests' which convert 2D screen space
// coordinates into 3D world coordinates based on intersection of the points
// with geometry in the world. This can be used to estimate the location of
// line keypoints in 3D space assuming a flat ground plane.

#ifndef PROJECT_GUIDELINE_UTIL_HIT_TEST_UTIL_H_
#define PROJECT_GUIDELINE_UTIL_HIT_TEST_UTIL_H_

#include "absl/status/statusor.h"
#include "Eigen/Core"
#include "project_guideline/camera/camera_model.h"
#include "project_guideline/util/transformation.h"

namespace guideline::util {

// HitResult saves the result of a camera hit test for a given pixel location
// in image coordinate space. `hit_pose` contains the position of the pixel
// in the world coordinate space. `his_distance` is the distance (meters)
// between camera and the `hit_pose`. `confidence` defines the accuracy of the
// the hit test. Typically larger the distance more is the position error.
struct HitResult {
  const util::Transformation hit_pose;
  const float hit_distance;
  const float confidence;

  HitResult(util::Transformation hit_pose, float hit_distance)
      : hit_pose(hit_pose), hit_distance(hit_distance), confidence(1.0) {}
  HitResult(util::Transformation hit_pose, float hit_distance,
            float confidence)
      : hit_pose(hit_pose),
        hit_distance(hit_distance),
        confidence(confidence) {}
};

util::Transformation CreateSyntheticGroundPlane(
    util::Transformation& camera_pose, float camera_height_meters);

absl::StatusOr<HitResult> HitTestPlane(
    const Eigen::Vector2d& pixel, const util::Transformation& world_T_plane,
    const util::Transformation& world_T_camera,
    const camera::CameraModel& camera_model);

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_HIT_TEST_UTIL_H_
