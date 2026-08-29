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

#ifndef PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_WORLD_MODEL_H_
#define PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_WORLD_MODEL_H_

#include "Eigen/Core"

namespace guideline::environment {

// A model-derived occupancy prior expressed in world coordinates.
//
// Modern video, segmentation, and depth foundation models can provide
// high-level evidence that a point in the runner's clearance zone is risky even
// when frame-local depth is noisy or sparse. Keeping this as a small data
// contract lets model-specific code live outside the safety-critical occupancy
// map while still giving the map a way to fuse learned risk estimates with the
// geometric point cloud.
struct WorldModelOccupancyPrior {
  WorldModelOccupancyPrior(const Eigen::Vector2d& position, float confidence,
                           float occupancy_weight)
      : position(position),
        confidence(confidence),
        occupancy_weight(occupancy_weight) {}

  Eigen::Vector2d position;
  float confidence = 0.f;
  float occupancy_weight = 0.f;
};

}  // namespace guideline::environment

#endif  // PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_WORLD_MODEL_H_
