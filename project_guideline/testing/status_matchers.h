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

#ifndef PROJECT_GUIDELINE_TESTING_STATUS_MATCHERS_H_
#define PROJECT_GUIDELINE_TESTING_STATUS_MATCHERS_H_

#include <memory>  // keep include

#include "gtest/gtest.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "project_guideline/util/status.h"

namespace guideline::testing::internal {
inline const absl::Status& GetStatus(const absl::Status& status) {
  return status;
}

template <typename T>
inline const absl::Status& GetStatus(const absl::StatusOr<T>& status) {
  return status.status();
}
}  // namespace guideline::testing::internal

#define GL_ASSERT_OK(expr) GL_ASSERT_OK_IMPL(GL_STATUS_UNIQUE(_status_), expr)

#define GL_ASSERT_OK_IMPL(status, expr)                          \
  auto status = ::guideline::testing::internal::GetStatus(expr); \
  ASSERT_TRUE(status.ok()) << status;

#define GL_EXPECT_OK(expr) GL_EXPECT_OK_IMPL(GL_STATUS_UNIQUE(_status_), expr)

#define GL_EXPECT_OK_IMPL(status, expr)                          \
  auto status = ::guideline::testing::internal::GetStatus(expr); \
  EXPECT_TRUE(status.ok()) << status;

#define GL_ASSERT_OK_AND_ASSIGN(lhs, rexpr) \
  GL_ASSERT_OK_AND_ASSIGN_IMPL(GL_STATUS_UNIQUE(_status_or_), lhs, rexpr)

#define GL_ASSERT_OK_AND_ASSIGN_IMPL(statusor, lhs, rexpr) \
  auto statusor = (rexpr);                                 \
  ASSERT_TRUE(statusor.ok()) << statusor.status();         \
  lhs = std::move(statusor.value());

#endif  // PROJECT_GUIDELINE_TESTING_STATUS_MATCHERS_H_
