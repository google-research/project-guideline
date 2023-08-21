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

#ifndef PROJECT_GUIDELINE_UTIL_EMBEDDED_FILE_TOC_H_
#define PROJECT_GUIDELINE_UTIL_EMBEDDED_FILE_TOC_H_

#include <cstddef>

namespace guideline::util {

struct EmbeddedFileToc {
 const char* name;
 const char* data;
 size_t size;
 unsigned char md5digest[16] = {};
};

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_EMBEDDED_FILE_TOC_H_
