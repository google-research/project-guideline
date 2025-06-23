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

#include "project_guideline/util/averaging_filter.h"

#include "gtest/gtest.h"
#include "absl/time/time.h"

namespace guideline::util {
namespace {

constexpr float kFloatTolerance = 1e-5;

TEST(AveragingFilter, FilterFloat) {
  AveragingFilter<float> filter(absl::Seconds(4));

  EXPECT_EQ(0, filter.average());

  absl::Time start;
  EXPECT_NEAR(1.1, filter.filter(start + absl::Seconds(100), 1.1),
              kFloatTolerance);
  filter.filter(start + absl::Seconds(101), 2.2);
  filter.filter(start + absl::Seconds(102), 3.3);
  EXPECT_NEAR(2.75, filter.filter(start + absl::Seconds(103), 4.4),
              kFloatTolerance);

  // Drops the first value (1.1).
  EXPECT_NEAR(3.85, filter.filter(start + absl::Seconds(105), 5.5),
              kFloatTolerance);

  EXPECT_NEAR(6.6, filter.filter(start + absl::Seconds(120), 6.6),
              kFloatTolerance);
  EXPECT_NEAR(0, filter.filter(start + absl::Seconds(121), -6.6),
              kFloatTolerance);
}

TEST(AveragingFilter, FilterInt) {
  AveragingFilter<int> filter(absl::Seconds(4));

  EXPECT_EQ(0, filter.average());

  absl::Time start;
  EXPECT_NEAR(1, filter.filter(start + absl::Seconds(100), 1), kFloatTolerance);
  filter.filter(start + absl::Seconds(101), 2);
  filter.filter(start + absl::Seconds(102), 3);
  EXPECT_NEAR(2, filter.filter(start + absl::Seconds(103), 4), kFloatTolerance);
}

TEST(LatchingAveragingFilter, LatchFloat) {
  LatchingAveragingFilter<float> latch(/*threshold=*/0.4,
                                       /*window=*/absl::Seconds(4),
                                       /*min_interval=*/absl::Seconds(2));

  EXPECT_FALSE(latch.state());

  absl::Time start;
  EXPECT_FALSE(latch.filter(start + absl::Seconds(100), 1.0));
  EXPECT_FALSE(latch.filter(start + absl::Seconds(101), 1.0));
  EXPECT_TRUE(latch.filter(start + absl::Seconds(102), 1.0));
  EXPECT_TRUE(latch.filter(start + absl::Seconds(103), 0));
  EXPECT_TRUE(latch.filter(start + absl::Seconds(104), 0));
  EXPECT_FALSE(latch.filter(start + absl::Seconds(105), 0));
}

}  // namespace
}  // namespace guideline::util
