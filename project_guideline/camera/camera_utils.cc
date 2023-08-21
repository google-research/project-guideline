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

#include "project_guideline/camera/camera_utils.h"

namespace guideline::camera {

void GlCameraProjectionMatrix(const CameraModel& model, double near, double far,
                              Eigen::Matrix4d& out) {
  // scale = near / [fx, fy]
  Eigen::Vector2d scale =
      model.pinhole_params().head<2>().cwiseInverse() * near;
  Eigen::Vector2d size(model.image_width() * scale.x(),
                       model.image_height() * scale.y());

  // offset = (scale * [cx, cy]) - [width/2, height/2]
  Eigen::Vector2d offset =
      (scale.cwiseProduct(model.pinhole_params().segment<2>(2))) - (size / 2.0);

  // Y points down
  offset.y() = -offset.y();

  out.setZero();
  out(0, 0) = 2 * near / size.x();
  out(1, 1) = 2 * near / size.y();
  out(0, 2) = -2 * offset.x() / size.x();
  out(1, 2) = -2 * offset.y() / size.y();
  out(2, 2) = (far + near) / (near - far);
  out(2, 3) = 2.0f * far * near / (near - far);
  out(3, 2) = -1.0f;
}

}  // namespace guideline::camera
