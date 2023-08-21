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

#include "project_guideline/environment/occupancy_map.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/synchronization/mutex.h"
#include "project_guideline/environment/obstacle_utils.h"

namespace guideline::environment {

using depth::Point3D;
using ::Eigen::Vector2d;
using util::Transformation;

namespace {
constexpr int kLogSeconds = 5;
}  // namespace

std::unique_ptr<OccupancyMap> OccupancyMap::Create(
    const OccupancyMapOptions& options) {
  CHECK_EQ(options.frame_based_occupancy_map_options()
                   .clearance_zone_options()
                   .width() %
               2,
           0)
      << "OccupancyMap::Create: Only support clearance zone width divisible "
         "by 2.";
  return absl::WrapUnique(
      new OccupancyMap(options.frame_based_occupancy_map_options()));
}

absl::StatusOr<std::vector<std::pair<Vector2d, int>>>
OccupancyMap::ComputeOccupancyMap(
    const std::vector<Point3D>& point_cloud,
    const Transformation& human_position_direction) {
  if (point_cloud.empty()) {
    return absl::FailedPreconditionError(
        "OccupancyMap::ComputeOccupancyMap: Not point cloud found.");
  }

  // Compute the clearance zone based on human position and direction.
  // This is used to save search space and storage for computing occupancy map.
  absl::flat_hash_map<std::pair<float, float>, int> occupancy_grids =
      GetClearanceZone(options_.clearance_zone_options().width(),
                       options_.clearance_zone_options().depth(),
                       human_position_direction);

  float clearance_zone_top = human_position_direction.p().z() +
                             options_.clearance_zone_options().top();
  float clearance_zone_bottom = human_position_direction.p().z() +
                                options_.clearance_zone_options().bottom();

  // Compute the occupancy map within the clearance zone, based on
  // the 3D point cloud.
  for (const auto& point : point_cloud) {
    if (point.confidence <= options_.point_confidence_threshold()) {
      continue;
    }
    if (point.coordinate.z() < clearance_zone_bottom ||
        point.coordinate.z() > clearance_zone_top) {
      continue;
    }
    const std::pair<float, float> closest_grid(round(point.coordinate.x()),
                                               round(point.coordinate.y()));
    if (occupancy_grids.contains(closest_grid)) {
      ++occupancy_grids[closest_grid];
    }
  }

  std::vector<std::pair<Vector2d, int>> occupancy_map;
  for (auto it = occupancy_grids.begin(); it != occupancy_grids.end(); ++it) {
    int occupancy = it->second;
    if (it->second < options_.occupancy_threshold()) {
      occupancy = 0;
    }
    occupancy_map.emplace_back(Vector2d(it->first.first, it->first.second),
                               occupancy);
  }

  return occupancy_map;
}

void OccupancyMap::UpdateOccupancyMap(
    const std::vector<Point3D>& point_cloud,
    const Transformation& human_position_direction) {
  auto new_occupancy_map =
      ComputeOccupancyMap(point_cloud, human_position_direction);
  if (!new_occupancy_map.ok()) {
    LOG_EVERY_N_SEC(WARNING, kLogSeconds)
        << new_occupancy_map.status().message();
    return;
  }
  absl::MutexLock lock(&occupancy_map_lock_);
  occupancy_map_ = std::move(*new_occupancy_map);
}

std::vector<std::pair<Vector2d, int>> OccupancyMap::GetOccupancyMap() {
  absl::MutexLock lock(&occupancy_map_lock_);
  return occupancy_map_;
}

}  // namespace guideline::environment
