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

#ifndef PROJECT_GUIDELINE_LOGGING_FILE_GUIDELINE_LOGGER_H_
#define PROJECT_GUIDELINE_LOGGING_FILE_GUIDELINE_LOGGER_H_

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "project_guideline/logging/guideline_logger.h"
#include "project_guideline/util/image.h"
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

namespace guideline::logging {

class FileGuidelineLogger : public GuidelineLogger {
 public:
  static absl::StatusOr<std::unique_ptr<FileGuidelineLogger>> Create(
      std::string output_dir, bool log_intermediate = false,
      DebugLogLevel log_level = DebugLogLevel::kWarning);

  ~FileGuidelineLogger() override;

  void LogEventInternal(GuidelineLogEvent& event) override
      ABSL_LOCKS_EXCLUDED(mutex_);

  void LogDepth(const util::Image& depth, const util::Image& conf,
                int64_t timestamp) override ABSL_LOCKS_EXCLUDED(mutex_);

  void Close() override ABSL_LOCKS_EXCLUDED(mutex_);

  bool GetLogIntermediate() override;

 protected:
  void LogDebugMessageInternal(DebugLogLevel level, const std::string& tag,
                               const std::string& message) override
      ABSL_LOCKS_EXCLUDED(mutex_);

 private:
  explicit FileGuidelineLogger(int fd, std::string output_dir,
                               bool log_intermediate, DebugLogLevel log_level);

  int fd_;
  std::string output_dir_;
  bool log_intermediate_;
  bool closed_ ABSL_GUARDED_BY(mutex_);
  std::unique_ptr<google::protobuf::io::FileOutputStream> file_output_stream_
      ABSL_GUARDED_BY(mutex_);
  std::unique_ptr<google::protobuf::io::CodedOutputStream> coded_output_stream_
      ABSL_GUARDED_BY(mutex_);
  absl::Mutex mutex_;
};

}  // namespace guideline::logging

#endif  // PROJECT_GUIDELINE_LOGGING_FILE_GUIDELINE_LOGGER_H_
