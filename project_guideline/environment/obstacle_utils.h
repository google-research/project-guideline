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

#ifndef PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_OBSTACLE_UTILS_H_
#define PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_OBSTACLE_UTILS_H_

#include <utility>

#include "absl/container/flat_hash_map.h"
#include "project_guideline/util/transformation.h"

namespace guideline::environment {

// Returns the partitioned grids within clearance zone which are used to compute
// the occupancy map.
// Assume human's current position is (0, 0), X-axis is to the human's right,
// Y-axis is towards human's current direction, and Z-axis is upwords pointing
// to sky, the clearance zone is defined as
// [-width/2:with/2, -1:depth, bottom:top] in meters.
absl::flat_hash_map<std::pair<float, float>, int> GetClearanceZone(
    uint32_t width_meters, uint32_t depth_meters,
    const util::Transformation& human_position_direction);

}  // namespace guideline::environment

#endif  // PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_OBSTACLE_UTILS_H_
