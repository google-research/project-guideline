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

#ifndef PROJECT_GUIDELINE_ENVIRONMENT_CONTROL_SIGNAL_H_
#define PROJECT_GUIDELINE_ENVIRONMENT_CONTROL_SIGNAL_H_

#include <optional>

#include "Eigen/Core"
#include "project_guideline/proto/control_signal_data.pb.h"

namespace guideline::environment {

// Control signal used to convey directions to the user based on current
// position, line shape/direction, and environmental features.
//
// The sign of the float values indicate the direction of the motion.
// For lateral movement negative is left and positive is right. For rotational
// movement negative is clockwise and positive is counterclockwise.
struct ControlSignal {
  bool stop = false;

  // Lateral distance tangential from the line.
  float lateral_movement_meters = 0.;

  // Desired user rotational movement based on user position and direction of
  // upcoming line.
  float rotation_movement_degrees = 0.;

  // World coordinates of an upcoming turn point, if any.
  std::optional<Eigen::Vector3d> turn_point = std::nullopt;

  // Angle in degrees representing sharpness of the turn indicated by
  // turn_point.
  float turn_angle_degrees = 0.;

  // Distance in meters to an upcoming turn indicated by turn_point.
  float turn_point_distance_meters = 0.;

  // Indicates whether there may be an obstacle in the vicinity ahead of the
  // user.
  bool obstacle_ahead = false;

  // Indicates whether this control signal is only conveying the presence of
  // obstacles and not giving any directional navigation signals.
  bool obstacle_only_mode = false;

  // The speed of the user in meters per second.
  float speed = 0.f;

  // The direction of the user based on the velocity. If the user is
  // sufficiently still then this direction is based on the camera orientation.
  Eigen::Vector3d velocity_direction = {0., 0., 0.};

  // A target point on the guideline to which the user is being guided.
  std::optional<Eigen::Vector3d> target_point = std::nullopt;

  // The desired rotational movement required to align the user with the
  // target_point based on the user's velocity_direction.
  float target_rotation_movement_degrees = 0.;

  // The point on guideline that is nearest to the user.
  std::optional<Eigen::Vector3d> closest_guideline_point = std::nullopt;

  // A point on the guideline some distance ahead of the closest point for the
  // user to follow.
  std::optional<Eigen::Vector3d> guideline_follow_point = std::nullopt;

  // Points on the lateral movement line that represent the edges of the track.
  std::optional<Eigen::Vector3d> left_edge_point = std::nullopt;
  std::optional<Eigen::Vector3d> right_edge_point = std::nullopt;

  ControlSignalData extra_data;
};

}  // namespace guideline::environment

#endif  // PROJECT_GUIDELINE_ENVIRONMENT_CONTROL_SIGNAL_H_
