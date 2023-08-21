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

#include "project_guideline/logging/file_guideline_logger.h"

#include <fcntl.h>
#include <string.h>

#include <cstring>
#include <filesystem>  // NOLINT
#include <memory>
#include <string>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "project_guideline/proto/guideline_log.pb.h"
#include "project_guideline/util/image_io.h"
#include <google/protobuf/util/time_util.h>
#include <google/protobuf/wire_format_lite.h>

namespace guideline::logging {

absl::StatusOr<std::unique_ptr<FileGuidelineLogger>>
FileGuidelineLogger::Create(std::string output_dir, bool log_intermediate,
                            DebugLogLevel log_level) {
  const std::string output_file = absl::StrCat(output_dir, "/guideline_log.pb");
  int fd = open(output_file.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd <= 0) {
    return absl::InternalError(absl::StrCat("Cannot open ", output_file));
  }

  return absl::WrapUnique(
      new FileGuidelineLogger(fd, output_dir, log_intermediate, log_level));
}

FileGuidelineLogger::FileGuidelineLogger(int fd, std::string output_dir,
                                         bool log_intermediate,
                                         DebugLogLevel log_level)
    : GuidelineLogger(log_level),
      fd_(fd),
      output_dir_(output_dir),
      log_intermediate_(log_intermediate),
      closed_(false),
      file_output_stream_(std::make_unique<google::protobuf::io::FileOutputStream>(fd)),
      coded_output_stream_(std::make_unique<google::protobuf::io::CodedOutputStream>(
          file_output_stream_.get())) {}

FileGuidelineLogger::~FileGuidelineLogger() { Close(); }

void FileGuidelineLogger::LogEventInternal(GuidelineLogEvent& event) {
  absl::MutexLock lock(&mutex_);

  if (closed_) {
    return;
  }

  *event.mutable_timestamp() = google::protobuf::util::TimeUtil::NanosecondsToTimestamp(
      absl::ToUnixNanos(absl::Now()));
  coded_output_stream_->WriteTag(google::protobuf::internal::WireFormatLite::MakeTag(
      GuidelineLog::kEventFieldNumber,
      google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED));
  coded_output_stream_->WriteVarint32(event.ByteSizeLong());
  event.SerializeToCodedStream(coded_output_stream_.get());
}

void FileGuidelineLogger::LogDepth(const util::Image& depth,
                                   const util::Image& conf, int64_t timestamp) {
  absl::MutexLock lock(&mutex_);

  if (closed_) {
    return;
  }

  const std::string image_name =
      absl::StrCat(std::to_string(timestamp), ".png");
  const std::string depth_dir = absl::StrCat(output_dir_, "/depth/");
  const std::string conf_dir = absl::StrCat(output_dir_, "/confidence/");

  if (!std::filesystem::exists(depth_dir)) {
    std::filesystem::create_directories(depth_dir);
    std::filesystem::permissions(
        depth_dir,
        std::filesystem::perms::owner_all | std::filesystem::perms::group_all,
        std::filesystem::perm_options::add);
  }

  if (!std::filesystem::exists(conf_dir)) {
    std::filesystem::create_directories(conf_dir);
    std::filesystem::permissions(
        depth_dir,
        std::filesystem::perms::owner_all | std::filesystem::perms::group_all,
        std::filesystem::perm_options::add);
  }

  CHECK_OK(util::SaveImage(depth, absl::StrCat(depth_dir, image_name)));
  CHECK_OK(util::SaveImage(conf, absl::StrCat(conf_dir, image_name)));
}

void FileGuidelineLogger::LogDebugMessageInternal(const DebugLogLevel level,
                                                  const std::string& tag,
                                                  const std::string& message) {
  // TODO: log to proto
}

void FileGuidelineLogger::Close() {
  absl::MutexLock lock(&mutex_);

  if (closed_) {
    return;
  }

  coded_output_stream_.reset();

  if (!file_output_stream_->Close()) {
    int error_no = file_output_stream_->GetErrno();
    LOG(ERROR) << "Failed to close FileOutputStream: errno = " << error_no
               << " (" << strerror(error_no) << ")";
  }

  file_output_stream_.reset();

  close(fd_);

  closed_ = true;
}

bool FileGuidelineLogger::GetLogIntermediate() { return log_intermediate_; }

}  // namespace guideline::logging
