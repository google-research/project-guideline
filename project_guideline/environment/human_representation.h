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

#ifndef PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_HUMAN_REPRESENTATION_H_
#define PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_HUMAN_REPRESENTATION_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "project_guideline/environment/object.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/transformation.h"

namespace guideline::environment {

// The window of time over which to compute the runner's velocity by comparing
// runner position at the start/end of this window.
// TODO: This is temporary value. Update with more appropriate value after
// initial experimentation.
static const absl::Duration kDefaultVelocityWindow = absl::Milliseconds(200);

struct CameraPose {
  util::Transformation transformation;
  int64_t timestamp_us;

  CameraPose(util::Transformation transformation, int64_t timestamp_us)
      : transformation(transformation), timestamp_us(timestamp_us) {}
};

class CameraPoseBasedHuman : public Object {
 public:
  static absl::StatusOr<std::unique_ptr<Object>> Create(
      const HumanRepresentationOptions& options);
  void UpdatePositionAndDirection(const util::Transformation& transformation,
                                  int64_t timestamp_us) override;
  absl::StatusOr<util::Transformation> CurrentPositionAndDirection()
      const override;
  absl::StatusOr<float> CurrentSpeed() const override;
  absl::StatusOr<::Eigen::Vector3d> VelocityVector() const override;
  // Returns list of cleared timestamps. We use timestamp as key for indexing
  // entries. This is useful for clearing the entries in other relevant
  // representations like the guideline aggregator.
  absl::Status ClearEntries(std::optional<size_t> num_entries) override;
  size_t NumEntries() const override;
  absl::StatusOr<float> ShapeAndSize() const override;

 private:
  explicit CameraPoseBasedHuman(const CameraPoseBasedHumanOptions options,
                                const absl::Duration velocity_window)
      : options_(options), velocity_window_(velocity_window) {}
  const CameraPoseBasedHumanOptions options_;
  const absl::Duration velocity_window_;
  mutable absl::Mutex trajectory_lock_;
  std::deque<CameraPose> trajectory_ ABSL_GUARDED_BY(trajectory_lock_);
};

}  // namespace guideline::environment

#endif  // PROJECT_GUIDELINE_ENVIRONMENT_REPRESENTATION_HUMAN_REPRESENTATION_H_
