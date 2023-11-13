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

#include "project_guideline/camera/cv_camera_model.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>  // keep include
#include "Eigen/Core"
#include "project_guideline/camera/camera_model.h"

namespace guideline::camera {

static const cv::Mat kZeroVec3Mat = cv::Mat::zeros(1, 3, CV_64F);

CvCameraModel::CvCameraModel(int image_width, int image_height,
                             const Eigen::Vector4d& pinhole_params,
                             const Eigen::VectorXd& distortion_coeffs)
    : CameraModel(image_width, image_height, pinhole_params) {
  cv::eigen2cv(intrinsic_matrix(), intrinsic_matrix_mat_);
  cv::eigen2cv(distortion_coeffs, distortion_coeffs_mat_);
}

bool CvCameraModel::PointToPixel(const Eigen::Vector3d& camera_t_point,
                                 Eigen::Vector2d& pixel) const {
  if (camera_t_point.z() <= 0.0) {
    // Point is behind the camera.
    return false;
  }
  cv::Mat camera_t_point_mat(1, 1, CV_64FC3, (void*)camera_t_point.data());
  cv::Mat image_points;
  // Project the point in camera coordinate frame to image pixel.
  cv::projectPoints(camera_t_point_mat, /*rvec=*/kZeroVec3Mat,
                    /*tvec=*/kZeroVec3Mat, intrinsic_matrix_mat_,
                    distortion_coeffs_mat_, image_points);
  pixel << image_points.at<double>(0), image_points.at<double>(1);
  return pixel.allFinite();
}

bool CvCameraModel::PixelToRay(const Eigen::Vector2d& pixel,
                               Eigen::Vector3d& ray) const {
  cv::Mat pixel_mat(1, 1, CV_64FC2, (void*)pixel.data());
  cv::Mat point;
  // This projects the pixel into camera coordinate frame for z = 1.
  cv::undistortPoints(pixel_mat, point, intrinsic_matrix_mat_,
                      distortion_coeffs_mat_);
  ray << point.at<double>(0), point.at<double>(1), 1.0;
  return ray.allFinite();
}

}  // namespace guideline::camera
