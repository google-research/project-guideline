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

#include "project_guideline/environment/human_representation.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "absl/log/check.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/util/status.h"
#include <google/protobuf/util/time_util.h>

namespace guideline::environment {

using ::Eigen::Vector3d;
using ::Eigen::Vector4d;
using util::Transformation;

absl::StatusOr<std::unique_ptr<Object>> CameraPoseBasedHuman::Create(
    const HumanRepresentationOptions& options) {
  CHECK_GT(options.camera_pose_based_human_options().max_history_to_keep(), 1);

  absl::Duration velocity_window = kDefaultVelocityWindow;

  if (options.camera_pose_based_human_options().has_velocity_window()) {
    velocity_window =
        absl::Milliseconds(google::protobuf::util::TimeUtil::DurationToMilliseconds(
            options.camera_pose_based_human_options().velocity_window()));
    CHECK_GT(velocity_window, absl::ZeroDuration());
  }

  std::unique_ptr<Object> base_ptr;
  std::unique_ptr<CameraPoseBasedHuman> derived_ptr;
  derived_ptr = absl::WrapUnique(new CameraPoseBasedHuman(
      options.camera_pose_based_human_options(), velocity_window));
  base_ptr = std::move(derived_ptr);
  return base_ptr;
}

size_t CameraPoseBasedHuman::NumEntries() const {
  absl::MutexLock lock(&trajectory_lock_);
  return trajectory_.size();
}

absl::Status CameraPoseBasedHuman::ClearEntries(
    std::optional<size_t> num_entries) {
  absl::MutexLock lock(&trajectory_lock_);
  size_t num_entries_value;
  if (num_entries.has_value()) {
    num_entries_value = num_entries.value();
  } else {
    num_entries_value = trajectory_.size();
  }

  if (num_entries_value > trajectory_.size()) {
    return absl::FailedPreconditionError(
        "ArcoreTrackingBasedHuman::ClearEntries: More entries to clear than "
        "available in the trajectory.");
  }

  for (size_t i = 0; i < num_entries_value; ++i) {
    trajectory_.pop_front();
  }
  return absl::OkStatus();
}

void CameraPoseBasedHuman::UpdatePositionAndDirection(
    const Transformation& transformation, int64_t timestamp_us) {
  absl::MutexLock lock(&trajectory_lock_);
  if (trajectory_.size() >= options_.max_history_to_keep()) {
    trajectory_.pop_front();
  }
  trajectory_.push_back(CameraPose(transformation, timestamp_us));
}

absl::StatusOr<Transformation>
CameraPoseBasedHuman::CurrentPositionAndDirection() const {
  absl::MutexLock lock(&trajectory_lock_);
  if (trajectory_.empty()) {
    return absl::NotFoundError(
        "CameraPoseBasedHuman::CurrentPositionAndDirection: Trajectory is "
        "empty.");
  } else {
    return trajectory_.back().transformation;
  }
}

absl::StatusOr<float> CameraPoseBasedHuman::CurrentSpeed() const {
  GL_ASSIGN_OR_RETURN(auto velocity_vector, VelocityVector());
  return velocity_vector.norm();
}

absl::StatusOr<Vector3d> CameraPoseBasedHuman::VelocityVector() const {
  absl::MutexLock lock(&trajectory_lock_);
  if (trajectory_.empty()) {
    return absl::NotFoundError("Trajectory is empty");
  }
  auto b = trajectory_.back();

  std::optional<CameraPose> start;
  absl::Duration duration = absl::ZeroDuration();
  for (int i = trajectory_.size() - 2; i >= 0; --i) {
    auto a = trajectory_[i];
    duration = absl::Microseconds(b.timestamp_us - a.timestamp_us);
    if (duration >= velocity_window_) {
      start = a;
      break;
    }
  }

  if (!start.has_value()) {
    return absl::NotFoundError("Not enough trajectory measurements");
  }

  // The resulting vector will be in meters / second.
  return (b.transformation.p() - start->transformation.p()) /
         absl::ToDoubleSeconds(duration);
}

absl::StatusOr<float> CameraPoseBasedHuman::ShapeAndSize() const {
  return absl::UnimplementedError(
      "CameraPoseBasedHuman::ShapeAndSize: Method not yet implemented.");
}

}  // namespace guideline::environment
