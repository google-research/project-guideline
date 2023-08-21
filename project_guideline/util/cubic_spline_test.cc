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

#include "project_guideline/util/cubic_spline.h"

#include <cmath>

#include "gtest/gtest.h"

namespace guideline::util {
namespace {

constexpr float k2Pi = M_PI * 2;
constexpr float kAllowedNumericDeviation = 1e-6f;

constexpr float kXOffset = -1.f;
constexpr float kCubicTerm = 2.f;
constexpr float kQuadraticTerm = -3.f;
constexpr float kLinearTerm = 4.f;
constexpr float kConstTerm = -5.f;

TEST(CubicFunctionTest, Constructors) {
  CubicFunction function_a;
  EXPECT_EQ(0.f, function_a.x_offset());
  EXPECT_EQ(0.f, function_a.cubic_term());
  EXPECT_EQ(0.f, function_a.quadratic_term());
  EXPECT_EQ(0.f, function_a.linear_term());
  EXPECT_EQ(0.f, function_a.absolute_term());

  CubicFunction function_b(kCubicTerm, kQuadraticTerm, kLinearTerm, kConstTerm);
  EXPECT_EQ(0.f, function_b.x_offset());
  EXPECT_EQ(kCubicTerm, function_b.cubic_term());
  EXPECT_EQ(kQuadraticTerm, function_b.quadratic_term());
  EXPECT_EQ(kLinearTerm, function_b.linear_term());
  EXPECT_EQ(kConstTerm, function_b.absolute_term());

  CubicFunction function_c(kXOffset, kCubicTerm, kQuadraticTerm, kLinearTerm,
                           kConstTerm);
  EXPECT_EQ(kXOffset, function_c.x_offset());
  EXPECT_EQ(kCubicTerm, function_c.cubic_term());
  EXPECT_EQ(kQuadraticTerm, function_c.quadratic_term());
  EXPECT_EQ(kLinearTerm, function_c.linear_term());
  EXPECT_EQ(kConstTerm, function_c.absolute_term());
}

TEST(CubicFunctionTest, GettersAndSetters) {
  CubicFunction function;
  function.set_x_offset(kXOffset);
  function.set_cubic_term(kCubicTerm);
  function.set_quadratic_term(kQuadraticTerm);
  function.set_linear_term(kLinearTerm);
  function.set_absolute_term(kConstTerm);

  EXPECT_EQ(kXOffset, function.x_offset());
  EXPECT_EQ(kCubicTerm, function.cubic_term());
  EXPECT_EQ(kQuadraticTerm, function.quadratic_term());
  EXPECT_EQ(kLinearTerm, function.linear_term());
  EXPECT_EQ(kConstTerm, function.absolute_term());
}

TEST(CubicFunctionTest, ValuesAsExpected) {
  CubicFunction function(kXOffset, kCubicTerm, kQuadraticTerm, kLinearTerm,
                         kConstTerm);
  EXPECT_EQ(kConstTerm, function.GetValue(-kXOffset));
  EXPECT_EQ(-14.f, function.GetValue(0.f));
  const float kExpectedAtTen = 1000.f * kCubicTerm + 100.f * kQuadraticTerm +
                               10 * kLinearTerm + kConstTerm;
  EXPECT_EQ(kExpectedAtTen, function.GetValue(10.f - kXOffset));
}

TEST(CubicFunctionTest, SlopesAsExpected) {
  // Construct two cubic functions: 'slope_function' is the first derivative of
  // 'function'. Make sure slopes of 'function' and values of 'slope_function'
  // coincide.
  CubicFunction function(kXOffset, kCubicTerm, kQuadraticTerm, kLinearTerm,
                         kConstTerm);
  CubicFunction slope_function(kXOffset, 0.f, 3.f * kCubicTerm,
                               2.f * kQuadraticTerm, kLinearTerm);
  EXPECT_EQ(slope_function.GetValue(-kXOffset), function.GetSlope(-kXOffset));
  EXPECT_EQ(slope_function.GetValue(0.f), function.GetSlope(0.f));
  EXPECT_EQ(slope_function.GetValue(10.f), function.GetSlope(10.f));
}

TEST(CubicFunctionTest, CurvaturesAsExpected) {
  // Construct two cubic functions: 'curvature_function' is the second
  // derivative of 'function'. Make sure curvatures of 'function' and values of
  // 'curvature_function' coincide.
  CubicFunction function(kXOffset, kCubicTerm, kQuadraticTerm, kLinearTerm,
                         kConstTerm);
  CubicFunction curvature_function(kXOffset, 0.f, 0.f, 6.f * kCubicTerm,
                                   2.f * kQuadraticTerm);

  EXPECT_EQ(curvature_function.GetValue(-kXOffset),
            function.GetCurvature(-kXOffset));
  EXPECT_EQ(curvature_function.GetValue(0.f), function.GetCurvature(0.f));
  EXPECT_EQ(curvature_function.GetValue(10.f), function.GetCurvature(10.f));
}

class CubicSplineTest
    : public testing::TestWithParam<CubicSpline::CubicSplineMode> {};

INSTANTIATE_TEST_SUITE_P(CubicSplineModesTest, CubicSplineTest,
                         ::testing::Values(CubicSpline::kNatural,
                                           CubicSpline::kClamped,
                                           CubicSpline::kPeriodic));

// Two points need to be added to the spline before any interpolation can be
// performed or any segment function can be obtained.
TEST_P(CubicSplineTest, TwoPointsRequired) {
  CubicSpline bad_spline(GetParam());
  float y;
  // For now, the spline should do nothing.
  EXPECT_EQ(0, bad_spline.GetSegmentCount());
  EXPECT_EQ(CubicSpline::kNotEnoughPointsError,
            bad_spline.Interpolate(1.f, &y));
  CubicFunction segment_function;
  EXPECT_EQ(CubicSpline::kNotEnoughPointsError,
            bad_spline.GetSegmentFunction(0, &segment_function));

  // Add one point.
  EXPECT_EQ(CubicSpline::kNoError, bad_spline.AddPoint(0.f, 0.f));
  EXPECT_EQ(0, bad_spline.GetSegmentCount());
  EXPECT_EQ(CubicSpline::kNotEnoughPointsError,
            bad_spline.Interpolate(1.f, &y));
  EXPECT_EQ(CubicSpline::kNotEnoughPointsError,
            bad_spline.GetSegmentFunction(0, &segment_function));

  // Try to add another point at x == 0.
  EXPECT_EQ(CubicSpline::kAlreadyDefinedError, bad_spline.AddPoint(0.f, 1.f));

  // Add a second point.
  EXPECT_EQ(CubicSpline::kNoError, bad_spline.AddPoint(1.f, 1.f));
  EXPECT_EQ(1, bad_spline.GetSegmentCount());

  CubicSpline::SplineError expected = (GetParam() == CubicSpline::kPeriodic)
                                          ? CubicSpline::kNotPeriodic
                                          : CubicSpline::kNoError;
  EXPECT_EQ(expected, bad_spline.Interpolate(1.f, &y));
  EXPECT_EQ(expected, bad_spline.GetSegmentFunction(0, &segment_function));
}

// Add points in permuted x-order. The points should be sorted by the spline.
TEST_P(CubicSplineTest, PointsSortedCorrectly) {
  CubicSpline some_spline(GetParam());
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(1.f, 1.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(11.f, 11.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(2.f, 2.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(0.f, 0.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(8.f, 8.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(9.f, 9.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(4.f, 4.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(5.f, 5.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(10.f, 10.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(7.f, 7.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(3.f, 3.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(6.f, 6.f));
  EXPECT_EQ(CubicSpline::kAlreadyDefinedError, some_spline.AddPoint(0.f, 0.f));
  EXPECT_EQ(CubicSpline::kAlreadyDefinedError, some_spline.AddPoint(1.f, 1.f));
  EXPECT_EQ(CubicSpline::kAlreadyDefinedError, some_spline.AddPoint(2.f, 2.f));
  EXPECT_EQ(CubicSpline::kAlreadyDefinedError, some_spline.AddPoint(3.f, 3.f));
  EXPECT_EQ(CubicSpline::kAlreadyDefinedError, some_spline.AddPoint(4.f, 4.f));
  EXPECT_EQ(CubicSpline::kAlreadyDefinedError,
            some_spline.AddPoint(11.f, 11.f));
  const int kPointCount = 12;
  EXPECT_EQ(kPointCount, some_spline.GetPointCount());
  for (int i = 0; i < kPointCount; ++i) {
    Eigen::Vector2f point;
    EXPECT_EQ(CubicSpline::kNoError, some_spline.GetPoint(i, &point));
    EXPECT_EQ(static_cast<float>(i), point(0));
    EXPECT_EQ(static_cast<float>(i), point(1));
  }
}

// Verify that the bounds of the spline are controlled correctly.
TEST_P(CubicSplineTest, BoundsCorrect) {
  CubicSpline some_spline(GetParam());
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(0.f, 0.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(2.f, 1.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(3.f, -1.f));
  EXPECT_EQ(CubicSpline::kNoError, some_spline.AddPoint(4.f, 0.f));
  EXPECT_EQ(3, some_spline.GetSegmentCount());
  EXPECT_EQ(4, some_spline.GetPointCount());
  CubicFunction segment_function;
  EXPECT_EQ(CubicSpline::kIndexOutOfBoundsError,
            some_spline.GetSegmentFunction(-1, &segment_function));
  EXPECT_EQ(CubicSpline::kIndexOutOfBoundsError,
            some_spline.GetSegmentFunction(5, &segment_function));

  // Linear interpolation for out-of-bounds points.
  float y;
  EXPECT_EQ(CubicSpline::kNoError, some_spline.Interpolate(-1.f, &y));
  EXPECT_NEAR(-0.5f, y, kAllowedNumericDeviation);
  EXPECT_EQ(CubicSpline::kNoError, some_spline.Interpolate(5.f, &y));
  EXPECT_NEAR(1.f, y, kAllowedNumericDeviation);
}

// The interpolation of 5 points on a line should be a line.
TEST_P(CubicSplineTest, LinearSpline) {
  CubicSpline linear_spline(GetParam());
  // When constructing a periodic spline, set the slope to 0.f to actually get a
  // periodic function.
  float slope = (GetParam() == CubicSpline::kPeriodic) ? 0.f : 2.f;
  // Try to clamp the slopes at the end. That should give an error if the spline
  // was not set to be in clamped mode.
  CubicSpline::SplineError result = linear_spline.ClampSlopes(slope, slope);
  if (GetParam() == CubicSpline::kClamped) {
    EXPECT_EQ(CubicSpline::kNoError, result);
  } else {
    EXPECT_EQ(CubicSpline::kNotClamped, result);
  }
  EXPECT_EQ(CubicSpline::kNoError, linear_spline.AddPoint(0.f, 0.f * slope));
  EXPECT_EQ(CubicSpline::kNoError, linear_spline.AddPoint(2.f, 2.f * slope));
  EXPECT_EQ(CubicSpline::kNoError, linear_spline.AddPoint(3.f, 3.f * slope));
  EXPECT_EQ(CubicSpline::kNoError, linear_spline.AddPoint(4.f, 4.f * slope));
  EXPECT_EQ(CubicSpline::kNoError, linear_spline.AddPoint(10.f, 10.f * slope));
  CubicFunction segment_function;
  for (int i = 0; i < linear_spline.GetSegmentCount(); ++i) {
    EXPECT_EQ(CubicSpline::kNoError,
              linear_spline.GetSegmentFunction(i, &segment_function));
    EXPECT_NEAR(0.f, segment_function.cubic_term(), kAllowedNumericDeviation);
    EXPECT_NEAR(0.f, segment_function.quadratic_term(),
                kAllowedNumericDeviation);
    EXPECT_NEAR(slope, segment_function.linear_term(),
                kAllowedNumericDeviation);
    Eigen::Vector2f point;
    EXPECT_EQ(CubicSpline::kNoError, linear_spline.GetPoint(i, &point));
    EXPECT_NEAR(point(1), segment_function.absolute_term(),
                kAllowedNumericDeviation);
    EXPECT_NEAR(-point(0), segment_function.x_offset(),
                kAllowedNumericDeviation);
  }
}

// Expect the coefficients of a simple example spline to match hand-calculated
// results.
TEST_P(CubicSplineTest, ThreePointSpline) {
  CubicSpline cubic_spline(GetParam());
  CubicSpline::SplineError result =
      cubic_spline.ClampSlopes(9.f / 4.f, 3.f / 4.f);
  if (GetParam() == CubicSpline::kClamped) {
    EXPECT_EQ(CubicSpline::kNoError, result);
  } else {
    EXPECT_EQ(CubicSpline::kNotClamped, result);
  }
  EXPECT_EQ(CubicSpline::kNoError, cubic_spline.AddPoint(0.f, 1.f));
  EXPECT_EQ(CubicSpline::kNoError, cubic_spline.AddPoint(1.f, 3.f));
  EXPECT_EQ(CubicSpline::kNoError, cubic_spline.AddPoint(2.f, 4.f));
  CubicFunction first_segment;
  CubicFunction second_segment;
  // The following tests don't make sense for a periodic spline.
  if (GetParam() != CubicSpline::kPeriodic) {
    EXPECT_EQ(CubicSpline::kNoError,
              cubic_spline.GetSegmentFunction(0, &first_segment));
    EXPECT_EQ(CubicSpline::kNoError,
              cubic_spline.GetSegmentFunction(1, &second_segment));
    EXPECT_NEAR(1.f, first_segment.absolute_term(), kAllowedNumericDeviation);
    EXPECT_NEAR(9.f / 4.f, first_segment.linear_term(),
                kAllowedNumericDeviation);
    EXPECT_NEAR(0.f, first_segment.quadratic_term(), kAllowedNumericDeviation);
    EXPECT_NEAR(-1.f / 4.f, first_segment.cubic_term(),
                kAllowedNumericDeviation);
    EXPECT_EQ(0.f, first_segment.x_offset());
    EXPECT_NEAR(3.f, second_segment.absolute_term(), kAllowedNumericDeviation);
    EXPECT_NEAR(3.f / 2.f, second_segment.linear_term(),
                kAllowedNumericDeviation);
    EXPECT_NEAR(-3.f / 4.f, second_segment.quadratic_term(),
                kAllowedNumericDeviation);
    EXPECT_NEAR(1.f / 4.f, second_segment.cubic_term(),
                kAllowedNumericDeviation);
    EXPECT_EQ(-1.f, second_segment.x_offset());
  }
}

TEST_P(CubicSplineTest, SineFitted) {
  const int kPointCount = 11;
  // Fit a sine curve with (kPointCount - 1) segments.
  CubicSpline sine_spline(GetParam());
  CubicSpline::SplineError result = sine_spline.ClampSlopes(1.f, 1.f);
  if (GetParam() == CubicSpline::kClamped) {
    EXPECT_EQ(CubicSpline::kNoError, result);
  } else {
    EXPECT_EQ(CubicSpline::kNotClamped, result);
  }
  for (int i = 0; i < kPointCount - 1; ++i) {
    float x = static_cast<float>(i) / (kPointCount - 1) * k2Pi;
    EXPECT_EQ(CubicSpline::kNoError, sine_spline.AddPoint(x, std::sin(x)));
  }
  // Set the y-value for the last point manually to 0.f, because otherwise, the
  // spline will be detected as non-periodic.
  EXPECT_EQ(CubicSpline::kNoError, sine_spline.AddPoint(k2Pi, 0.f));

  // Interpolate datapoints and expect the deviation from the original curve to
  // be low.
  const int kInterpolationCount = 100;
  float y = 0.f;
  float kAllowedDeviation = 1e-3f;
  for (int i = 0; i <= kInterpolationCount; ++i) {
    float x = static_cast<float>(i) / kInterpolationCount * k2Pi;
    EXPECT_EQ(CubicSpline::kNoError, sine_spline.Interpolate(x, &y));
    EXPECT_NEAR(std::sin(x), y, kAllowedDeviation);
  }
}

}  // namespace
}  // namespace guideline::util
