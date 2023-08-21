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

// Utilities for working with absl::Status.

#ifndef PROJECT_GUIDELINE_UTIL_STATUS_H_
#define PROJECT_GUIDELINE_UTIL_STATUS_H_

#include "absl/base/optimization.h"
#include "absl/log/check.h"

#define GL_STATUS_UNIQUE(name) GL_STATUS_CONCAT(name, __COUNTER__)
#define GL_STATUS_CONCAT(x, y) GL_STATUS_CONCAT_IMPL(x, y)
#define GL_STATUS_CONCAT_IMPL(x, y) x##y

#define GL_RETURN_IF_ERROR(expr)                         \
  {                                                      \
    auto status = (expr);                                \
    if (ABSL_PREDICT_FALSE(!status.ok())) return status; \
  }

#define GL_ASSIGN_OR_RETURN(lhs, rexpr) \
  GL_ASSIGN_OR_RETURN_IMPL(GL_STATUS_UNIQUE(_status_or_), lhs, rexpr)

#define GL_ASSIGN_OR_RETURN_IMPL(statusor, lhs, rexpr)              \
  auto statusor = (rexpr);                                          \
  if (ABSL_PREDICT_FALSE(!statusor.ok())) return statusor.status(); \
  lhs = std::move(statusor.value());

#define GL_CHECK_OK_AND_ASSIGN(lhs, rexpr) \
  GL_CHECK_OK_AND_ASSIGN_IMPL(GL_STATUS_UNIQUE(_status_or_), lhs, rexpr)

#define GL_CHECK_OK_AND_ASSIGN_IMPL(statusor, lhs, rexpr) \
  auto statusor = (rexpr);                                \
  CHECK_OK(statusor);                                     \
  lhs = std::move(statusor.value());

#endif  // PROJECT_GUIDELINE_UTIL_STATUS_H_
