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

#ifndef PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_OBJECT_H_
#define PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_OBJECT_H_

#include <optional>

#include "absl/status/statusor.h"
#include "Eigen/Core"
#include "project_guideline/util/transformation.h"

namespace guideline::environment {

// General interface for objects in the environment (including humans and
// obstacles).
class Object {
 public:
  virtual ~Object() = default;

  // Updates the position and direction of the object as observed at the given
  // timestamp.
  virtual void UpdatePositionAndDirection(
      const util::Transformation& transformation, int64_t timestamp) = 0;

  // Gets the current position and direction of the object based on the
  // last call to UpdatePositionAndDirection().
  virtual absl::StatusOr<util::Transformation> CurrentPositionAndDirection()
      const = 0;

  // Gets the current speed of the object in meters/second.
  virtual absl::StatusOr<float> CurrentSpeed() const = 0;

  // TODO: Define appropriate return type for shape and size.
  virtual absl::StatusOr<float> ShapeAndSize() const = 0;

  // Gets the current velocity vector of the object, in meters/second.
  virtual absl::StatusOr<::Eigen::Vector3d> VelocityVector() const = 0;

  // Clears the given number of recent position entries, oldest first.
  virtual absl::Status ClearEntries(std::optional<size_t> num_entries) = 0;

  // Gets the current number of tracked position/direction entries.
  virtual size_t NumEntries() const = 0;
};

}  // namespace guideline::environment

#endif  // PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_OBJECT_H_
