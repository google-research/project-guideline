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

#ifndef PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_OBSTACLE_H_
#define PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_OBSTACLE_H_

#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "project_guideline/environment/object.h"
#include "project_guideline/util/transformation.h"

namespace guideline::environment {

struct ObstacleEntry {
  int64_t timestamp;
  // TODO(b/221263550): Add entries required to describe an obstacle at a
  // timestamp.
};

class Obstacle : public Object {
  // This class could be inherited by different types of obstacles.
  // E.g.: dynamic vs statis obstacles.
  // TODO(b/221263550): Add additional appropriate non virtual methods.
 public:
  void UpdatePositionAndDirection(
      const util::Transformation& transformation,
      int64_t timestamp) override {
    position_ = transformation;
  }

  absl::StatusOr<util::Transformation>
  CurrentPositionAndDirection() const override {
    return position_;
  }

  absl::StatusOr<float> CurrentSpeed() const override { return 0.0f; }

  absl::StatusOr<::Eigen::Vector3d> VelocityVector() const override {
    // TODO(b/221263550): Implement velocity
    return ::Eigen::Vector3d(0., 0., 0.);
  }

  absl::Status ClearEntries(std::optional<size_t> num_entries) override {
    // TODO(b/221263550): Implement after tracking obstacle across timestamps.
    return absl::OkStatus();
  }

  size_t NumEntries() const override { return 1; }

  absl::StatusOr<float> ShapeAndSize() const override { return 0.f; }

 private:
  util::Transformation position_;
  std::deque<ObstacleEntry> trajectory_;
};

class ObstacleAggregator {
 public:
  // TODO(b/221263550): Add appropriate virtual methods.
  absl::StatusOr<std::vector<Obstacle>> GetObstacles() {
    return std::vector<Obstacle>();
  }
  void AddOrUpdateObstacle();

 private:
  absl::Mutex obstacles_lock_;
  std::deque<Obstacle> obstacles_ ABSL_GUARDED_BY(obstacles_lock_);
};

}  // namespace guideline::environment

#endif  // PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_OBSTACLE_H_
