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

#include "project_guideline/logging/guideline_logger.h"

#include <string>

#ifdef __ANDROID__
#include <android/log.h>
#endif

namespace guideline::logging {

void GuidelineLogger::LogEvent(GuidelineLogEvent event) {
  LogEventInternal(event);
  absl::MutexLock lock(&mutex_);
  if (log_event_callback_.has_value()) {
    (*log_event_callback_)(event);
  }
}

void GuidelineLogger::LogDebugMessage(const DebugLogLevel level,
                                      const std::string& tag,
                                      const std::string& message) {
#ifdef __ANDROID__
  auto android_level = ANDROID_LOG_INFO;
  switch (level) {
    case kError:
      android_level = ANDROID_LOG_ERROR;
      break;
    case kWarning:
      android_level = ANDROID_LOG_WARN;
      break;
    case kDebug:
      android_level = ANDROID_LOG_DEBUG;
      break;
    case kVerbose:
      android_level = ANDROID_LOG_VERBOSE;
      break;
    case kInfo:
      // fall-through
    default:
      android_level = ANDROID_LOG_INFO;
  }
  // For Android always log the message without checking log level enabled,
  // since Android has its own mechanism for dynamically changing logging
  // level by tag, i.e. 'adb shell setprop log.tag.MyTag VERBOSE'.
  __android_log_print(android_level, tag.c_str(), "%s", message.c_str());
#endif
  if (!IsLogLevelEnabled(level)) {
    return;
  }
  LogDebugMessageInternal(level, tag, message);
  absl::MutexLock lock(&mutex_);
  if (debug_message_callback_.has_value()) {
    (*debug_message_callback_)(level, message);
  }
}

void GuidelineLogger::SetDebugMessageCallback(
    const DebugMessageCallback& callback) {
  absl::MutexLock lock(&mutex_);
  debug_message_callback_ = callback;
}

void GuidelineLogger::SetLogEventCallback(const LogEventCallback& callback) {
  absl::MutexLock lock(&mutex_);
  log_event_callback_ = callback;
}

bool GuidelineLogger::IsLogLevelEnabled(const DebugLogLevel level) {
  return level <= log_level_;
}

}  // namespace guideline::logging
