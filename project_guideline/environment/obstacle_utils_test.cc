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

#include "project_guideline/environment/obstacle_utils.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "Eigen/Core"
#include "project_guideline/util/transformation.h"

namespace guideline::environment {
namespace {

using ::Eigen::Vector3d;
using ::Eigen::Vector4d;
using ::testing::Pair;
using ::testing::UnorderedElementsAre;
using util::Transformation;

TEST(GetClearanceZoneTest, TestClearanceZone1) {
  Transformation human_position_direction({0, 0, -0.707, -0.707}, {0, 0, 1});

  EXPECT_THAT(GetClearanceZone(/*width_meters=*/4, /*depth_meters=*/2,
                               human_position_direction),
              UnorderedElementsAre(
                  Pair(std::pair(0, 0), 0), Pair(std::pair(1, 0), 0),
                  Pair(std::pair(2, 0), 0), Pair(std::pair(-1, 0), 0),
                  Pair(std::pair(-2, 0), 0), Pair(std::pair(0, 1), 0),
                  Pair(std::pair(1, 1), 0), Pair(std::pair(2, 1), 0),
                  Pair(std::pair(-1, 1), 0), Pair(std::pair(-2, 1), 0),
                  Pair(std::pair(0, 2), 0), Pair(std::pair(1, 2), 0),
                  Pair(std::pair(2, 2), 0), Pair(std::pair(-1, 2), 0),
                  Pair(std::pair(-2, 2), 0), Pair(std::pair(0, -1), 0),
                  Pair(std::pair(1, -1), 0), Pair(std::pair(2, -1), 0),
                  Pair(std::pair(-1, -1), 0), Pair(std::pair(-2, -1), 0)));
}

TEST(GetClearanceZoneTest, TestClearanceZone2) {
  Transformation human_position_direction({0.707, 0, 0.707, 0}, {0, 1, 1});

  EXPECT_THAT(
      GetClearanceZone(/*width_meters=*/2, /*depth_meters=*/3,
                       human_position_direction),
      UnorderedElementsAre(Pair(std::pair(-1, 0), 0), Pair(std::pair(-1, 1), 0),
                           Pair(std::pair(-1, 2), 0), Pair(std::pair(0, 0), 0),
                           Pair(std::pair(0, 1), 0), Pair(std::pair(0, 2), 0),
                           Pair(std::pair(1, 0), 0), Pair(std::pair(1, 1), 0),
                           Pair(std::pair(1, 2), 0), Pair(std::pair(2, 0), 0),
                           Pair(std::pair(2, 1), 0), Pair(std::pair(2, 2), 0),
                           Pair(std::pair(3, 0), 0), Pair(std::pair(3, 1), 0),
                           Pair(std::pair(3, 2), 0)));
}

}  // namespace
}  // namespace guideline::environment
