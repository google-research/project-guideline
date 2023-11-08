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

#include "project_guideline/util/lerp.h"

#include "gtest/gtest.h"

namespace guideline::util {
namespace {

TEST(LerpTest, Lerp) {
  // In range
  ASSERT_EQ(100, Lerp(0., 0., 1., 100., 200.));
  ASSERT_EQ(150, Lerp(0.5, 0., 1., 100., 200.));
  ASSERT_EQ(200, Lerp(1., 0., 1., 100., 200.));

  // Out of range
  ASSERT_EQ(50, Lerp(-0.5, 0., 1., 100., 200.));
  ASSERT_EQ(250, Lerp(1.5, 0., 1., 100., 200.));

  // Reversed range
  ASSERT_EQ(175, Lerp(0.25, 1., 0., 100., 200.));
  ASSERT_EQ(125, Lerp(0.75, 1., 0., 100., 200.));
  ASSERT_EQ(175, Lerp(0.25, 0., 1., 200., 100.));
  ASSERT_EQ(125, Lerp(0.75, 0., 1., 200., 100.));
}

TEST(LerpTest, ClampedLerp) {
  // In range
  ASSERT_EQ(100, ClampedLerp(0., 0., 1., 100., 200.));
  ASSERT_EQ(150, ClampedLerp(0.5, 0., 1., 100., 200.));
  ASSERT_EQ(200, ClampedLerp(1., 0., 1., 100., 200.));

  // Out of range
  ASSERT_EQ(100, ClampedLerp(-0.5, 0., 1., 100., 200.));
  ASSERT_EQ(200, ClampedLerp(1.5, 0., 1., 100., 200.));

  // Reversed out of range
  ASSERT_EQ(200, ClampedLerp(-0.5, 0., 1., 200., 100.));
  ASSERT_EQ(100, ClampedLerp(1.5, 0., 1., 200., 100.));

  // Reversed range
  ASSERT_EQ(175, ClampedLerp(-0.75, 0., -1., 100., 200.));
  ASSERT_EQ(200, ClampedLerp(-2., 0., -1., 100., 200.));
  ASSERT_EQ(100, ClampedLerp(5., 0., -1., 100., 200.));
}

TEST(LerpTest, LerpWithGamma) {
  // In range
  ASSERT_EQ(100, Lerp(0., 0., 1., 100., 200., 2.));
  ASSERT_EQ(106.25, Lerp(0.25, 0., 1., 100., 200., 2.));
  ASSERT_EQ(125, Lerp(0.5, 0., 1., 100., 200., 2.));
  ASSERT_EQ(156.25, Lerp(0.75, 0., 1., 100., 200., 2.));
  ASSERT_EQ(200, Lerp(1., 0., 1., 100., 200., 2.));

  ASSERT_EQ(100, Lerp(0., 0., 1., 100., 200., 0.5));
  ASSERT_EQ(150, Lerp(0.25, 0., 1., 100., 200., 0.5));
  ASSERT_EQ(200, Lerp(1., 0., 1., 100., 200., 0.5));

  // Out of range
  ASSERT_EQ(125, Lerp(-0.5, 0., 1., 100., 200., 2.));
  ASSERT_EQ(325, Lerp(1.5, 0., 1., 100., 200., 2.));

  // Reversed range
  ASSERT_EQ(193.75, Lerp(0.25, 0., 1., 200., 100., 2.));
  ASSERT_EQ(143.75, Lerp(0.75, 0., 1., 200., 100., 2.));
}

TEST(LerpTest, ClampedLerpWithGamma) {
  // In range
  ASSERT_EQ(100, ClampedLerp(0., 0., 1., 100., 200., 2.));
  ASSERT_EQ(106.25, ClampedLerp(0.25, 0., 1., 100., 200., 2.));
  ASSERT_EQ(125, ClampedLerp(0.5, 0., 1., 100., 200., 2.));
  ASSERT_EQ(156.25, ClampedLerp(0.75, 0., 1., 100., 200., 2.));
  ASSERT_EQ(200, ClampedLerp(1., 0., 1., 100., 200., 2.));

  // Out of range
  ASSERT_EQ(100, ClampedLerp(-0.5, 0., 1., 100., 200., 2.));
  ASSERT_EQ(200, ClampedLerp(1.5, 0., 1., 100., 200., 2.));

  // Reversed range
  ASSERT_EQ(193.75, ClampedLerp(0.25, 0., 1., 200., 100., 2.));
  ASSERT_EQ(143.75, ClampedLerp(0.75, 0., 1., 200., 100., 2.));
}

}  // namespace
}  // namespace guideline::util
