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

#ifndef PROJECT_GUIDELINE_LOGGING_DEBUG_SIGNAL_H_
#define PROJECT_GUIDELINE_LOGGING_DEBUG_SIGNAL_H_

#include <cstdint>
namespace guideline::logging {
struct DebugSignal {
  bool tracking;
  int64_t camera_pose_fps;
  int64_t camera_pose_lagging_frame;
  int64_t detection_fps;
  int64_t detection_lagging_frame;
  int64_t tracking_features_fps;
  int64_t tracking_features_lagging_frame;
};
}  // namespace guideline::logging

#endif  // PROJECT_GUIDELINE_LOGGING_DEBUG_SIGNAL_H_
