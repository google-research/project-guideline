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

// Defines the classes CubicFunction and CubicSpline.
// CubicFunction represents a simple cubic function with 4 coefficients. In
// order to facilitate moving this function along the x-axis, it also defines
// an x-offset, which is added to any input x before evaluating the function.
//
// CubicSpline defines an N-point cubic spline function. Coordinates that are
// added to the spline are sorted by x-coordinate. Fitting the spline (which
// also happens implicitly when trying to fetch segment functions) will cause
// a cubic spline to be layed over the added points.
// Note that a cubic spline function defines a mapping from x- to y- coordinates
// for the x-range in which the added points lie. Hence, this class does not
// allow to create arbitrary parametric curves like circles.

#ifndef PROJECT_GUIDELINE_UTIL_CUBIC_SPLINE_H_
#define PROJECT_GUIDELINE_UTIL_CUBIC_SPLINE_H_

#include <vector>

#include "Eigen/Core"

namespace guideline::util {

class CubicFunction {
 public:
  CubicFunction()
      : x_offset_(0.f),
        cubic_term_(0.f),
        quadratic_term_(0.f),
        linear_term_(0.f),
        absolute_term_(0.f) {}

  CubicFunction(float x_offset, float cubic_term, float quadratic_term,
                float linear_term, float absolute_term)
      : x_offset_(x_offset),
        cubic_term_(cubic_term),
        quadratic_term_(quadratic_term),
        linear_term_(linear_term),
        absolute_term_(absolute_term) {}

  CubicFunction(float cubic_term, float quadratic_term, float linear_term,
                float absolute_term)
      : x_offset_(0.f),
        cubic_term_(cubic_term),
        quadratic_term_(quadratic_term),
        linear_term_(linear_term),
        absolute_term_(absolute_term) {}

  float GetValue(float x) const {
    x += x_offset_;
    return ((x * cubic_term_ + quadratic_term_) * x + linear_term_) * x +
           absolute_term_;
  }

  float GetSlope(float x) const {
    x += x_offset_;
    return (3.f * x * cubic_term_ + 2.f * quadratic_term_) * x + linear_term_;
  }

  float GetCurvature(float x) const {
    x += x_offset_;
    return 6.f * x * cubic_term_ + 2.f * quadratic_term_;
  }

  float x_offset() const { return x_offset_; }

  void set_x_offset(float x_offset) { x_offset_ = x_offset; }

  float cubic_term() const { return cubic_term_; }

  void set_cubic_term(float cubic_term) { cubic_term_ = cubic_term; }

  float quadratic_term() const { return quadratic_term_; }

  void set_quadratic_term(float quadratic_term) {
    quadratic_term_ = quadratic_term;
  }

  float linear_term() const { return linear_term_; }

  void set_linear_term(float linear_term) { linear_term_ = linear_term; }

  float absolute_term() const { return absolute_term_; }

  void set_absolute_term(float absolute_term) {
    absolute_term_ = absolute_term;
  }

 private:
  float x_offset_;
  float cubic_term_;
  float quadratic_term_;
  float linear_term_;
  float absolute_term_;
};

class CubicSpline {
 public:
  // Defines the mode of the spline fitting.
  // kClamped:  Will fit a clamped spline to the given input data. That is, the
  //            spline will have user-defined first derivatives at the ends.
  //            The default slopes for the ends are 0.f and can be defined using
  //            the function ClampSlopes().
  // kNatural:  Will fit a natural spline to the given input data. That is, the
  //            second derivatives at the ends will be 0.
  // kPeriodic: Will fit a periodic spline to the given input data. That is,
  //            first and second derivatives at the ends of the spline will be
  //            equal to each other. In this mode, it is expected that the last
  //            y-coordinate is equal to the first y-coordinate
  enum CubicSplineMode { kClamped, kNatural, kPeriodic };

  enum SplineError {
    kNoError,
    kNotEnoughPointsError,
    kNumericProblemError,
    kIndexOutOfBoundsError,
    kAlreadyDefinedError,
    kNotPeriodic,
    kNotClamped,
    kOutsideError
  };

  explicit CubicSpline(CubicSplineMode mode);

  // Adds a points at the defined position. Points can be added in random order
  // of x, but remember this class defines a 'function', not a parametric curve.
  // Hence, coordinates will internally be sorted by their x-value. Adding two
  // points with the same x-coordinate will cause the function to return
  // kAlreadyDefined.
  SplineError AddPoint(float x, float y);

  // Interpolates the spline function value at position x. If the spline has not
  // yet been fitted to the current data, it will be fitted before calculating
  // y.
  SplineError Interpolate(float x, float* y);

  // Sets 'segment_function' to the interpolation result of the segment at
  // position 'index'. The spline will be fitted if it wasn't before.
  SplineError GetSegmentFunction(int index, CubicFunction* segment_function);

  SplineError GetPoint(int index, Eigen::Vector2f* point) const;

  // Returns the number of segments that will be fitted. This function will
  // not fit the spline but simply derive the number of segments from the
  // current number of points.
  int GetSegmentCount() const;

  int GetPointCount() const;

  // Clamp the slopes at the ends of the spline. Will return kNotClamped when
  // the spline is not initialized to kClamped.
  SplineError ClampSlopes(float start_slope, float end_slope);

  // Fit a cubic spline function to the added data points. This function will
  // return an error if there were less than 2 points added to the curve.
  SplineError Fit();

 protected:
  int FindSegment(float x) const;

 private:
  std::vector<Eigen::Vector2f> points_;
  std::vector<CubicFunction> segment_functions_;
  bool curve_fitted_;
  const CubicSplineMode mode_;
  float start_slope_;
  float end_slope_;
};

}  // namespace guideline::util

#endif  // PROJECT_GUIDELINE_UTIL_CUBIC_SPLINE_H_
