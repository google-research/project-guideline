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

#ifndef PROJECT_GUIDELINE_LOGGING_GUIDELINE_LOGGER_H_
#define PROJECT_GUIDELINE_LOGGING_GUIDELINE_LOGGER_H_

#include <functional>
#include <optional>
#include <string>

#include "absl/synchronization/mutex.h"
#include "project_guideline/proto/guideline_log.pb.h"
#include "project_guideline/util/image.h"

namespace guideline::logging {

enum DebugLogLevel { kError, kWarning, kInfo, kDebug, kVerbose };

using DebugMessageCallback =
    std::function<void(const DebugLogLevel level, const std::string& message)>;
using LogEventCallback = std::function<void(const GuidelineLogEvent& event)>;

class GuidelineLogger {
 public:
  explicit GuidelineLogger(DebugLogLevel log_level = DebugLogLevel::kWarning)
      : log_level_(log_level) {}
  virtual ~GuidelineLogger() = default;
  virtual void LogDepth(const util::Image& depth, const util::Image& conf,
                        int64_t timestamp) = 0;
  virtual void Close() = 0;
  virtual bool GetLogIntermediate() = 0;

  void LogEvent(GuidelineLogEvent event);
  void LogDebugMessage(DebugLogLevel level, const std::string& tag,
                       const std::string& message);
  void SetDebugMessageCallback(const DebugMessageCallback& callback);
  void SetLogEventCallback(const LogEventCallback& callback);
  bool IsLogLevelEnabled(DebugLogLevel level);

 protected:
  virtual void LogEventInternal(GuidelineLogEvent& event) = 0;
  virtual void LogDebugMessageInternal(DebugLogLevel level,
                                       const std::string& tag,
                                       const std::string& message) = 0;

 private:
  DebugLogLevel log_level_;
  absl::Mutex mutex_;
  std::optional<DebugMessageCallback> debug_message_callback_
      ABSL_GUARDED_BY(mutex_) = std::nullopt;
  std::optional<LogEventCallback> log_event_callback_ ABSL_GUARDED_BY(mutex_) =
      std::nullopt;
};

}  // namespace guideline::logging

#endif  // PROJECT_GUIDELINE_LOGGING_GUIDELINE_LOGGER_H_
