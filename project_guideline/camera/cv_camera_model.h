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

#include "project_guideline/camera/camera_model.h"

#ifndef PROJECT_GUIDELINE_CAMERA_CV_CAMERA_MODEL_H_
#define PROJECT_GUIDELINE_CAMERA_CV_CAMERA_MODEL_H_

#include <opencv2/core/mat.hpp>
#include "Eigen/Core"  // keep include

namespace guideline::camera {

// Camera model that uses OpenCV parameters and utilities.
// The distortion coefficients are optional and when omitted will be a pinhole
// model.
class CvCameraModel : public CameraModel {
 public:
  // pinhole_params: (fx, fy, cx, cy)
  // distortion_coeffs: (k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4[,τx,τy]]]]) of
  // 4, 5, 8, 12 or 14 elements. Can be set to [0, 0, 0, 0] for pinhole.
  CvCameraModel(
      int image_width, int image_height, const Eigen::Vector4d& pinhole_params,
      const Eigen::VectorXd& distortion_coeffs = Eigen::Vector4d::Zero());

  bool PointToPixel(const Eigen::Vector3d& camera_t_point,
                    Eigen::Vector2d& pixel) const override;

  bool PixelToRay(const Eigen::Vector2d& pixel,
                  Eigen::Vector3d& ray) const override;

 private:
  cv::Mat intrinsic_matrix_mat_;
  cv::Mat distortion_coeffs_mat_;
};

}  // namespace guideline::camera

#endif  // PROJECT_GUIDELINE_CAMERA_CV_CAMERA_MODEL_H_
