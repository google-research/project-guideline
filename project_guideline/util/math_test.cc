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

#include "project_guideline/util/math.h"

#include <limits>
#include <vector>

#include "gtest/gtest.h"

namespace guideline::util {
namespace {

TEST(Math, Mean) {
  std::vector<float> vf = {0};
  EXPECT_NEAR(Mean(vf), 0, std::numeric_limits<float>::epsilon());

  vf = {-4, 3.2, 0., 5.6};
  EXPECT_NEAR(Mean(vf), 1.2, 1e-5);

  vf = {1.0, std::numeric_limits<float>::infinity()};
  EXPECT_EQ(Mean(vf), std::numeric_limits<float>::infinity());

  std::vector<double> vd = {0};
  EXPECT_NEAR(Mean(vd), 0, std::numeric_limits<double>::epsilon());

  vd = {-4, 3.2, 0., 5.6};
  EXPECT_NEAR(Mean(vd), 1.2, 1e-5);

  vd = {1.0, std::numeric_limits<double>::infinity()};
  EXPECT_EQ(Mean(vd), std::numeric_limits<double>::infinity());
}

TEST(Math, SumOfSquares) {
  std::vector<float> vf = {0};
  EXPECT_NEAR(SumOfSquares(vf), 0, std::numeric_limits<float>::epsilon());

  vf = {-4, 3.2, 0., 5.6};
  EXPECT_NEAR(SumOfSquares(vf), 57.6, 1e-5);

  vf = {1.0, std::numeric_limits<float>::infinity()};
  EXPECT_EQ(SumOfSquares(vf), std::numeric_limits<float>::infinity());

  std::vector<double> vd = {0};
  EXPECT_NEAR(SumOfSquares(vd), 0, std::numeric_limits<double>::epsilon());

  vd = {-4, 3.2, 0., 5.6};
  EXPECT_NEAR(SumOfSquares(vd), 57.6, 1e-5);

  vd = {1.0, std::numeric_limits<double>::infinity()};
  EXPECT_EQ(SumOfSquares(vd), std::numeric_limits<double>::infinity());
}

TEST(Math, StandardDeviation) {
  std::vector<float> vf = {0};
  EXPECT_NEAR(StandardDeviation(vf), 0, std::numeric_limits<float>::epsilon());

  vf = {-4, 3.2, 0., 5.6};
  EXPECT_NEAR(StandardDeviation(vf), 3.6, 1e-5);

  vf = {1.0, std::numeric_limits<float>::infinity()};
  EXPECT_TRUE(std::isnan(StandardDeviation(vf)));

  std::vector<double> vd = {0};
  EXPECT_NEAR(StandardDeviation(vd), 0, std::numeric_limits<double>::epsilon());

  vd = {-4, 3.2, 0., 5.6};
  EXPECT_NEAR(StandardDeviation(vd), 3.6, 1e-5);

  vd = {1.0, std::numeric_limits<double>::infinity()};
  EXPECT_TRUE(std::isnan(StandardDeviation(vf)));
}

}  // namespace
}  // namespace guideline::util
