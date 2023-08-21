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

#include "project_guideline/util/file.h"

#include <fstream>
#include <iterator>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"

namespace guideline::util {

absl::Status ReadFileToString(const std::string& file_path, std::string& out) {
  std::ifstream ifs;
  ifs.open(file_path, std::ifstream::in | std::ifstream::binary);
  if (!ifs.is_open()) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Failed to open file: %s", file_path));
  }
  out.assign(std::istreambuf_iterator<char>(ifs),
             std::istreambuf_iterator<char>());
  if (ifs.fail()) {
    return absl::InternalError(
        absl::StrFormat("Failed to read file: %s", file_path));
  }
  return absl::OkStatus();
}

}  // namespace guideline::util
