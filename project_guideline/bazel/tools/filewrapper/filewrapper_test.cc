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

#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "absl/flags/flag.h"
#include "project_guideline/bazel/tools/filewrapper/test_embed_multiple.h"
#include "project_guideline/bazel/tools/filewrapper/test_embed_single.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/embedded_file_toc.h"
#include "project_guideline/util/file.h"

namespace guideline {
namespace {

using ::testing::IsNull;
using ::testing::StrEq;
using util::EmbeddedFileToc;

const char kTestDataPath[] =
    "/project_guideline/project_guideline/bazel/tools/filewrapper/testdata/";

TEST(FilewrapperTest, EmbedSingle) {
  const EmbeddedFileToc* toc = test_embed_single_create();

  EXPECT_EQ(test_embed_single_size(), 1);
  EXPECT_THAT(toc->name, StrEq("test1.bin"));
  EXPECT_EQ(toc->size, 1024);

  std::string contents;
  GL_ASSERT_OK(util::ReadFileToString(
      std::string(getenv("TEST_SRCDIR")) + kTestDataPath + "test1.bin",
      contents));
  EXPECT_THAT(std::string(toc->data, toc->size), StrEq(contents));

  ++toc;
  EXPECT_THAT(toc->name, ::testing::IsNull());
}

TEST(FilewrapperTest, EmbedMultiple) {
  const EmbeddedFileToc* toc = testns::test_embed_multiple_create();

  EXPECT_EQ(testns::test_embed_multiple_size(), 3);
  EXPECT_THAT(toc[0].name, StrEq("test1.bin"));
  EXPECT_EQ(toc[0].size, 1024);
  EXPECT_THAT(toc[1].name, StrEq("test2.bin"));
  EXPECT_EQ(toc[1].size, 2048);
  EXPECT_THAT(toc[2].name, StrEq("test3.bin"));
  EXPECT_EQ(toc[2].size, 4001);
  EXPECT_THAT(toc[3].name, IsNull());

  std::string contents;
  GL_ASSERT_OK(util::ReadFileToString(
      std::string(getenv("TEST_SRCDIR")) + kTestDataPath + "test1.bin",
      contents));
  EXPECT_THAT(std::string(toc[0].data, toc[0].size), StrEq(contents));

  GL_ASSERT_OK(util::ReadFileToString(
      std::string(getenv("TEST_SRCDIR")) + kTestDataPath + "test2.bin",
      contents));
  EXPECT_THAT(std::string(toc[1].data, toc[1].size), StrEq(contents));

  GL_ASSERT_OK(util::ReadFileToString(
      std::string(getenv("TEST_SRCDIR")) + kTestDataPath + "test3.bin",
      contents));
  EXPECT_THAT(std::string(toc[2].data, toc[2].size), StrEq(contents));
}

}  // namespace
}  // namespace guideline
