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

// Utilities for various 2D and 3D geometric operations.

#ifndef PROJECT_GUIDELINE_UTIL_GEOMETRY_H_
#define PROJECT_GUIDELINE_UTIL_GEOMETRY_H_

#include "Eigen/Core"      // keep include
#include "Eigen/Geometry"  // keep include
#include "project_guideline/util/transformation.h"

namespace guideline::util {

constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kRadiansToDegees = 1.0 / kDegreesToRadians;

// Represents the 3D axes with values corresponding to their Vector3d indices.
enum class Axis3 { kX = 0, kY = 1, kZ = 2 };

// Flattens a 3D vector into 2D space by removing the specified component.
inline Eigen::Vector3d Flatten2D(const Eigen::Vector3d& a, Axis3 flatten_axis) {
  Eigen::Vector3d a2(a);
  a2(static_cast<int>(flatten_axis)) = 0;
  return a2;
}

// Sets the given axis component of a vector to 0.
inline void ClearAxis(Eigen::Vector3d& a, Axis3 axis) {
  a(static_cast<int>(axis)) = 0;
}

inline double DegreesToRadians(double degrees) {
  return degrees * kDegreesToRadians;
}

inline double RadiansToDegrees(double radians) {
  return radians / kDegreesToRadians;
}

// Converts a quaternion in hamilton convention to JPL convention.
// See:
// JPL: http://www-users.cs.umn.edu/~trawny/Publications/Quaternions_3D.pdf
// Hamilton: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
inline Eigen::Vector4d HamiltonToJpl(const Eigen::Quaterniond& q) {
  return {-q.x(), -q.y(), -q.z(), q.w()};
}

// Converts a quaternion in JPL convention to hamilton convention.
// See:
// JPL: http://www-users.cs.umn.edu/~trawny/Publications/Quaternions_3D.pdf
// Hamilton: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
inline Eigen::Quaterniond JplToHamilton(const Eigen::Vector4d& jpl) {
  // Quaterniond constructor takes (w,x,y,z)
  return {jpl.w(), -jpl.x(), -jpl.y(), -jpl.z()};
}

// Computes the 2D distance between points, flattening the given axis.
double ComputeDistance2D(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                         Axis3 flatten_axis);

// Computes a 2D unit direction vector from src to dst, flattening the given
// axis.
Eigen::Vector3d ComputeUnitDirection2D(const Eigen::Vector3d& src,
                                       const Eigen::Vector3d& dst,
                                       Axis3 flatten_axis);

// Computes the angle between src and dst, flattening the given axis.
double ComputeAngle2D(const Eigen::Vector3d& src, const Eigen::Vector3d& dst,
                      Axis3 flatten_axis);

// Computes the forward direction vector based on a quaternion. This assumes
// MT coordinate space where identity quaternion corresponds to +Z direction.
Eigen::Vector3d ComputeForwardDirectionFromQuaternion(
    const Eigen::Quaterniond& q);

// Computes a perpendicular line pointing right from the direction vector from
// point src to dst, flattening the given axis.
Eigen::Vector3d ComputePerpendicularDirection2D(const Eigen::Vector3d& src,
                                                const Eigen::Vector3d& dst,
                                                Axis3 flatten_axis);

// Given a line from point line_src to line_dst which extends infinitely in both
// directions, computes the distance from point p to the closest point along
// that line (which will always be perpendicular). The distance is signed, with
// positive value indicating the point is to the right of the line (given
// direction from src to dst) and negative to the left. The distance is computed
// in 2D by flattening the given axis.
double ComputePerpendicularDistanceFromLine2D(const Eigen::Vector3d& p,
                                              const Eigen::Vector3d& line_src,
                                              const Eigen::Vector3d& line_dst,
                                              Axis3 flatten_axis);

// Creates a Transformation for a camera at position looking at look_at point.
Transformation LookAt(const Eigen::Vector3d& src,
                      const Eigen::Vector3d& look_at,
                      const Eigen::Vector3d& up);

// Computes yaw and pitch angles from the given Quaternion. Yaw is computed as
// rotation around the Z-axis, with Y direction being 0 yaw. Pitch is the angle
// from the XY plane.
Eigen::Vector2d ComputeYawPitch(const Eigen::Quaterniond& q);

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_GEOMETRY_H_
