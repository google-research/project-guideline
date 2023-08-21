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

#include "project_guideline/environment/obstacle_utils.h"

#include <utility>

#include "project_guideline/util/geometry.h"

namespace guideline::environment {

using ::Eigen::Vector3d;
using util::Transformation;

absl::flat_hash_map<std::pair<float, float>, int> GetClearanceZone(
    uint32_t width_meters, uint32_t depth_meters,
    const Transformation& human_position_direction) {
  const Vector3d& human_position = human_position_direction.p();
  Vector3d human_direction =
      util::ComputeForwardDirectionFromQuaternion(human_position_direction.q());

  human_direction.z() = 0;
  human_direction.normalize();

  // Compute the normal vector orthogonal to the current human direction.
  Vector3d orth_vector(human_direction.y(), -human_direction.x(), 0);

  // The key corresponds to the Coordinate2D, the value is initialized to 0
  // for accumulation of occupied points.
  absl::flat_hash_map<std::pair<float, float>, int> occupancy_grids;

  // Compute boundaries of the clearance zone, and partition into 1m x 1m grids.
  // TODO: support configurable grid size.
  for (int64_t i = -2; i <= 2 * depth_meters; ++i) {
    const Vector3d& steps_on_path = human_position + 0.5 * i * human_direction;
    for (uint32_t j = 0; j <= width_meters / 2; ++j) {
      occupancy_grids.emplace(
          std::pair<float, float>(
              round(steps_on_path.x() + orth_vector.x() * j),
              round(steps_on_path.y() + orth_vector.y() * j)),
          0);
      occupancy_grids.emplace(
          std::pair<float, float>(
              round(steps_on_path.x() - orth_vector.x() * j),
              round(steps_on_path.y() - orth_vector.y() * j)),
          0);
    }
  }
  return occupancy_grids;
}

}  // namespace guideline::environment
