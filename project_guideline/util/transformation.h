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

#ifndef PROJECT_GUIDELINE_UTIL_TRANSFORMATION_H_
#define PROJECT_GUIDELINE_UTIL_TRANSFORMATION_H_

#include <ostream>

#include "Eigen/Core"      // keep include
#include "Eigen/Geometry"  // keep include

namespace guideline::util {

typedef Eigen::Quaterniond Quaternion;
typedef Eigen::Vector3d Vector3;
typedef Eigen::Matrix4d Matrix4;

inline Quaternion IdentityQuaternion() { return Quaternion(1, 0, 0, 0); }

// A 3D transformation T = (q, p) where q is the rotational component and p
// is the translation.
// Multiplying the transformation by a vector has the operation:
//     T * x := (q * x) + p
// Multiplying (composing) two transformations has the operation:
//     T1 * T2 := T(T1.q * T2.q, (T1.q * T2.p) + T1.p)
class Transformation {
 public:
  Transformation() : q_(IdentityQuaternion()), p_(Vector3::Zero()) {}

  Transformation(const Quaternion& q)
      : q_(q.normalized()), p_(Vector3::Zero()) {}

  Transformation(const Vector3& p) : q_(IdentityQuaternion()), p_(p) {}

  Transformation(const Quaternion& q, const Vector3& p)
      : q_(q.normalized()), p_(p) {}

  static inline Transformation FromMatrix(const Eigen::Matrix4d& m) {
    return Transformation(Quaternion(m.topLeftCorner<3, 3>()),
                          Vector3(m.topRightCorner<3, 1>()));
  }

  // Convenience for returning an identity transformation.
  static Transformation Identity() { return Transformation(); }

  // Returns a hamilton quaternion representing the rotational part of this
  // transformation.
  const Quaternion& q() const { return q_; }

  // Returns a vector representing the translational part of this
  // transformation.
  const Vector3& p() const { return p_; }

  // Returns the inverse of this transformation.
  Transformation Inverse() const {
    const Quaternion q_inv = q_.inverse();
    return Transformation(q_inv, q_inv * -p_);
  }

  // Converts this transformation to a 4x4 affine matrix.
  Matrix4 ToMatrix4x4() const {
    Matrix4 m;
    m.template topLeftCorner<3, 3>() = q_.toRotationMatrix();
    m.template topRightCorner<3, 1>() = p_;
    m.template bottomLeftCorner<1, 3>().setZero();
    m(3, 3) = 1.0;
    return m;
  }

  Vector3 operator*(const Vector3& x) const { return q_ * x + p_; }

  Transformation operator*(const Quaternion& q) const {
    return Transformation(q_ * q, p_);
  }

  Transformation operator*(const Transformation& rhs) const {
    return Transformation(q_ * rhs.q_, this->operator*(rhs.p_));
  }

  Transformation operator+(const Vector3& p) const {
    return Transformation(q_, p_ + p);
  }

  bool operator==(const Transformation& rhs) const {
    return q_ == rhs.q_ && p_ == rhs.p_;
  }

  bool isApprox(const Transformation& rhs) const {
    return q_.isApprox(rhs.q_) && p_.isApprox(rhs.p_);
  }

  friend std::ostream& operator<<(std::ostream& output,
                                  const Transformation& transformation) {
    output << "q = [x: " << transformation.q().x()
           << ", y: " << transformation.q().y()
           << ", z: " << transformation.q().z()
           << ", w: " << transformation.q().w()
           << "], p = [x: " << transformation.p().x()
           << ", y: " << transformation.p().y()
           << ", z: " << transformation.p().z() << "]";
    return output;
  }

 private:
  Quaternion q_;
  Vector3 p_;
};

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_TRANSFORMATION_H_
