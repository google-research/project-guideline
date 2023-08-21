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

#ifndef PROJECT_GUIDELINE_MOTION_TRACKING_FEATURE_H_
#define PROJECT_GUIDELINE_MOTION_TRACKING_FEATURE_H_

#include <limits>

#include "Eigen/Core"

namespace guideline::motion {
struct TrackingFeature {
  TrackingFeature() = default;

  TrackingFeature(Eigen::Vector3f camera_t_feature, float confidence,
                  int32_t feature_id)
      : camera_t_feature(camera_t_feature),
        confidence(confidence),
        feature_id(feature_id) {}

  Eigen::Vector3f camera_t_feature = Eigen::Vector3f(0, 0, 0);
  float confidence = 0;
  int32_t feature_id = -1;
};
}  // namespace guideline::motion

#endif  // PROJECT_GUIDELINE_MOTION_TRACKING_FEATURE_H_
