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

#ifndef PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_OCCUPANCY_MAP_H_
#define PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_OCCUPANCY_MAP_H_

#include <memory>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "Eigen/Core"
#include "project_guideline/depth/point_cloud_util.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/transformation.h"

namespace guideline::environment {

// Occupancy map is a list of coordinate2D indicating the centroids of the
// occupied grids from bird-eye-view.
// More details can be found in doc go/guideline-obstacle-detection-v0
// The current occupancy map is frame-centric stateless. In the future versions,
// We will extend it to be stateful.
class OccupancyMap {
 public:
  static std::unique_ptr<OccupancyMap> Create(
      const OccupancyMapOptions& options);

  // Returns a sequence of Coordinate2D indicating the centroids of the
  // occupied grids from bird-eye-view. If returned sequency is empty, meaning
  // no obstacles within clearance zone.
  std::vector<std::pair<Eigen::Vector2d, int>> GetOccupancyMap();
  void UpdateOccupancyMap(const std::vector<depth::Point3D>& point_cloud,
                          const util::Transformation& human_position_direction);

 private:
  explicit OccupancyMap(const FrameBasedOccupancyMapOptions options)
      : options_(options) {}
  absl::StatusOr<std::vector<std::pair<Eigen::Vector2d, int>>>
  ComputeOccupancyMap(const std::vector<depth::Point3D>& point_cloud,
                      const util::Transformation& human_position_direction);
  absl::Mutex occupancy_map_lock_;
  std::vector<std::pair<Eigen::Vector2d, int>> occupancy_map_
      ABSL_GUARDED_BY(occupancy_map_lock_);
  const FrameBasedOccupancyMapOptions options_;
};

}  // namespace guideline::environment

#endif  // PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_OCCUPANCY_MAP_H_
