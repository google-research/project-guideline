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

#include "project_guideline/util/geometry.h"

#include <cmath>

#include "Eigen/Dense"  // keep include

namespace guideline::util {
using Eigen::Index;
using Eigen::Quaterniond;
using Eigen::Vector3d;

double ComputeDistance2D(const Vector3d& a, const Vector3d& b,
                         Axis3 flatten_axis) {
  Vector3d a2 = Flatten2D(a, flatten_axis);
  Vector3d b2 = Flatten2D(b, flatten_axis);
  return (b2 - a2).norm();
}

Eigen::Vector3d ComputeUnitDirection2D(const Vector3d& src, const Vector3d& dst,
                                       Axis3 flatten_axis) {
  Vector3d diff = dst - src;
  ClearAxis(diff, flatten_axis);
  diff.normalize();
  return diff;
}

double ComputeAngle2D(const Vector3d& src, const Vector3d& dst,
                      Axis3 flatten_axis) {
  Vector3d src2 = Flatten2D(src, flatten_axis);
  Vector3d dst2 = Flatten2D(dst, flatten_axis);
  Vector3d cross = src2.cross(dst2);
  double angle = std::atan2(cross.norm(), src2.dot(dst2));
  return cross[static_cast<int>(flatten_axis)] < 0 ? -angle : angle;
}

Eigen::Vector3d ComputeForwardDirectionFromQuaternion(const Quaterniond& q) {
  // In MT Coordinate space (Z-up, right-handed), Quaternion x,y,z,w = 0,0,0,1
  // corresponds to UnitZ direction. Use the quaternion to rotate the UnitZ
  // vector to obtain the direction vector.
  return q * Vector3d::UnitZ();
}

Eigen::Vector3d ComputePerpendicularDirection2D(const Vector3d& src,
                                                const Vector3d& dst,
                                                Axis3 flatten_axis) {
  Vector3d src2 = Flatten2D(src, flatten_axis);
  Vector3d dst2 = Flatten2D(dst, flatten_axis);

  // Make a unit vector in direction of axis we are ignoring, and cross it with
  // the direction vector to get a perpendicular vector on the same plane.
  Vector3d p = Vector3d::Zero();
  p[static_cast<int>(flatten_axis)] = 1;
  return (dst2 - src2).normalized().cross(p);
}

double ComputePerpendicularDistanceFromLine2D(const Eigen::Vector3d& p,
                                              const Eigen::Vector3d& line_src,
                                              const Eigen::Vector3d& line_dst,
                                              Axis3 flatten_axis) {
  Vector3d p2 = Flatten2D(p, flatten_axis);
  Vector3d line_src2 = Flatten2D(line_src, flatten_axis);
  Vector3d line_dst2 = Flatten2D(line_dst, flatten_axis);

  auto line_direction = line_dst2 - line_src2;
  double area = line_direction.cross(p2 - line_src2).sum();
  return area / line_direction.norm();
}

Transformation LookAt(const Eigen::Vector3d& src,
                      const Eigen::Vector3d& look_at,
                      const Eigen::Vector3d& up) {
  Eigen::Matrix3d r;

  // y = -up
  r.col(1) = -up;

  // Axis z toward look_at
  r.col(2) = (look_at - src);

  // x = y cross z
  r.col(0) = r.col(1).cross(r.col(2));

  // y = z cross x
  r.col(1) = r.col(2).cross(r.col(0));

  // Normalize axes
  r.col(0).normalize();
  r.col(1).normalize();
  r.col(2).normalize();

  return {Eigen::Quaterniond(r), src};
}

Eigen::Vector2d ComputeYawPitch(const Eigen::Quaterniond& q) {
  auto direction = ComputeForwardDirectionFromQuaternion(q);
  double yaw = atan2(direction.x(), direction.y());
  double pitch = asin(direction.z());
  return {yaw, pitch};
}

}  // namespace guideline::util