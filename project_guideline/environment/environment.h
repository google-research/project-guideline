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

#ifndef PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_ENVIRONMENT_H_
#define PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_ENVIRONMENT_H_

#include <memory>
#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "project_guideline/environment/guideline_aggregator.h"
#include "project_guideline/environment/object.h"
#include "project_guideline/environment/obstacle.h"
#include "project_guideline/environment/occupancy_map.h"
#include "project_guideline/environment/point_cloud.h"
#include "project_guideline/util/status.h"

namespace guideline::environment {

class Environment {
 public:
  static absl::StatusOr<std::unique_ptr<Environment>> Create(
      std::unique_ptr<Object> human,
      std::unique_ptr<GuidelineAggregator> guideline_aggregator,
      std::unique_ptr<PointCloud> point_cloud,
      std::unique_ptr<OccupancyMap> occupancy_map,
      std::unique_ptr<ObstacleAggregator> obstacle_aggregator) {
    return absl::WrapUnique(
        new Environment(std::move(human), std::move(guideline_aggregator),
                        std::move(point_cloud), std::move(occupancy_map),
                        std::move(obstacle_aggregator)));
  }

  absl::Status ClearAll() {
    // Environment reset.
    GL_RETURN_IF_ERROR(human_->ClearEntries(std::nullopt));
    GL_RETURN_IF_ERROR(guideline_aggregator_->ClearEntries(std::nullopt));
    // TODO(b/221263550): Add appropriate call to clear the obstacles.
    return absl::OkStatus();
  }

  std::unique_ptr<Object> human_;
  std::unique_ptr<GuidelineAggregator> guideline_aggregator_;
  std::unique_ptr<PointCloud> point_cloud_;
  std::unique_ptr<OccupancyMap> occupancy_map_;
  std::unique_ptr<ObstacleAggregator> obstacle_aggregator_;

 private:
  Environment(std::unique_ptr<Object> human,
              std::unique_ptr<GuidelineAggregator> guideline_aggregator,
              std::unique_ptr<PointCloud> point_cloud,
              std::unique_ptr<OccupancyMap> occupancy_map,
              std::unique_ptr<ObstacleAggregator> obstacle_aggregator)
      : human_(std::move(human)),
        guideline_aggregator_(std::move(guideline_aggregator)),
        point_cloud_(std::move(point_cloud)),
        occupancy_map_(std::move(occupancy_map)),
        obstacle_aggregator_(std::move(obstacle_aggregator)) {}
};
}  // namespace guideline::environment

#endif  // PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_ENVIRONMENT_H_
