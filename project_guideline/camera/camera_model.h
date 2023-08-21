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

#ifndef PROJECT_GUIDELINE_CAMERA_CAMERA_MODEL_H_
#define PROJECT_GUIDELINE_CAMERA_CAMERA_MODEL_H_

#include "Eigen/Core"  // keep include

namespace guideline::camera {

// Model for a camera at a particular image size that supports projecting
// points from camera coordinate space to pixel coordinates and the reverse.
// Model implementations will at least have camera pinhole parameters
// (fx, fy, cx, fy) and may include modeling for additional optical distortions
// such as radial and tangential.
class CameraModel {
 public:
  typedef Eigen::Matrix<double, 2, 3, Eigen::RowMajor> Matrix23d;

  CameraModel(int image_width, int image_height,
              const Eigen::Vector4d& pinhole_params)
      : image_width_(image_width),
        image_height_(image_height),
        pinhole_params_(pinhole_params) {}

  virtual ~CameraModel() = default;

  inline int image_width() const { return image_width_; }
  inline int image_height() const { return image_height_; }

  // Returns params for a pinhole model [fx, fy, cx, cy].
  inline const Eigen::Vector4d& pinhole_params() const {
    return pinhole_params_;
  }

  // Returns the intrinsic matrix based on the pinhole params.
  inline Eigen::Matrix3d intrinsic_matrix() const {
    Eigen::Matrix3d intrinsics;
    // [[fx, 0, cx],
    //  [0, fy, cy],
    //  [0, 0, 1]]
    intrinsics << pinhole_params_[0], 0, pinhole_params_[2], 0,
        pinhole_params_[1], pinhole_params_[3], 0, 0, 1;
    return intrinsics;
  }

  // Projects a point from camera coordinates to pixel coordinates.
  // Returns true if the projection succeeded. Note that this may return
  // true even if the pixel is outside the camera image dimensions.
  virtual bool PointToPixel(const Eigen::Vector3d& camera_t_point,
                            Eigen::Vector2d& pixel) const = 0;

  // Projects a point from pixel coordinates to camera coordinates.
  // Returns true if the projection succeeded.
  virtual bool PixelToRay(const Eigen::Vector2d& pixel,
                          Eigen::Vector3d& ray) const = 0;

 private:
  const int image_width_;
  const int image_height_;
  const Eigen::Vector4d pinhole_params_;
};

}  // namespace guideline::camera

#endif  // PROJECT_GUIDELINE_CAMERA_CAMERA_MODEL_H_
