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

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "project_guideline/logging/noop_guideline_logger.h"

namespace guideline::logging {
namespace {

TEST(GuidelineLogger, TestDebugLog) {
  std::optional<std::pair<DebugLogLevel, std::string>> last_message;
  const DebugMessageCallback callback =
      [&last_message](const DebugLogLevel level, const std::string& message) {
        last_message = {level, message};
      };

  auto logger = std::make_unique<NoopGuidelineLogger>();
  logger->SetDebugMessageCallback(callback);

  EXPECT_TRUE(logger->IsLogLevelEnabled(DebugLogLevel::kWarning));
  std::string expected_message = "warning message";
  logger->LogDebugMessage(DebugLogLevel::kWarning, "tag", expected_message);
  EXPECT_TRUE(last_message.has_value());
  EXPECT_EQ(last_message->first, DebugLogLevel::kWarning);
  EXPECT_EQ(last_message->second, expected_message);

  EXPECT_FALSE(logger->IsLogLevelEnabled(DebugLogLevel::kVerbose));
  last_message = std::nullopt;
  logger->LogDebugMessage(DebugLogLevel::kVerbose, "tag", "verbose");
  EXPECT_FALSE(last_message.has_value());
}

}  // namespace
}  // namespace guideline::logging
