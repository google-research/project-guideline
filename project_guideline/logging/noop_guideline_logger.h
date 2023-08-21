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

#ifndef PROJECT_GUIDELINE_LOGGING_NOOP_GUIDELINE_LOGGER_H_
#define PROJECT_GUIDELINE_LOGGING_NOOP_GUIDELINE_LOGGER_H_

#include <string>

#include "project_guideline/logging/guideline_logger.h"
#include "project_guideline/proto/guideline_log.pb.h"

namespace guideline::logging {

class NoopGuidelineLogger : public GuidelineLogger {
 public:
  void LogDepth(const util::Image& depth, const util::Image& conf,
                int64_t timestamp) override {}
  void Close() override {}
  bool GetLogIntermediate() override { return false; }

 protected:
  void LogEventInternal(GuidelineLogEvent& event) override {}
  void LogDebugMessageInternal(const DebugLogLevel level,
                               const std::string& tag,
                               const std::string& message) override {}
};

}  // namespace guideline::logging

#endif  // PROJECT_GUIDELINE_LOGGING_NOOP_GUIDELINE_LOGGER_H_
