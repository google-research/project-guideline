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

#include <algorithm>
#include <vector>

#include "Eigen/QR"

namespace guideline::util {

namespace {
using Eigen::Vector2f;
}  // namespace

CubicSpline::CubicSpline(CubicSplineMode mode)
    : curve_fitted_(false), mode_(mode), start_slope_(0.f), end_slope_(0.f) {}

CubicSpline::SplineError CubicSpline::AddPoint(float x, float y) {
  int segment = FindSegment(x);
  if (segment >= GetPointCount() - 1) {
    points_.push_back(Vector2f(x, y));
  } else {
    if (points_[segment + 1](0) == x) {
      return kAlreadyDefinedError;
    }
    if (segment < 0) {
      points_.insert(points_.begin(), Vector2f(x, y));
    } else {
      points_.insert(points_.begin() + segment + 1, Vector2f(x, y));
    }
  }
  curve_fitted_ = false;
  return kNoError;
}

CubicSpline::SplineError CubicSpline::Interpolate(float x, float* y) {
  if (!curve_fitted_) {
    CubicSpline::SplineError error = Fit();
    if (error != kNoError) {
      return error;
    }
  }
  int segment = FindSegment(x);
  // If the x-coordinate equals the first point in points_, FindSegment will
  // point to -1.
  if (segment == -1 && points_[0](0) == x) {
    segment = 0;
  }
  if (segment < 0 && segment_functions_.size() > 1) {
    float x1 = points_[0](0);
    float x2 = points_[1](0);
    float y1 = segment_functions_[0].GetValue(x1);
    float y2 = segment_functions_[1].GetValue(x2);
    *y = y1 + (((x - x1) / (x2 - x1)) * (y2 - y1));

  } else if (segment >= segment_functions_.size() &&
             segment_functions_.size() > 1) {
    int i1 = GetPointCount() - 2;
    int i2 = GetPointCount() - 1;
    float x1 = points_[i1](0);
    float x2 = points_[i2](0);
    int s1 = FindSegment(x1);
    int s2 = FindSegment(x2);
    float y1 = segment_functions_[s1].GetValue(x1);
    float y2 = segment_functions_[s2].GetValue(x2);
    *y = y1 + (((x - x1) / (x2 - x1)) * (y2 - y1));

  } else {
    segment = std::max(
        0, std::min(static_cast<int>(segment_functions_.size() - 1), segment));
    *y = segment_functions_[segment].GetValue(x);
  }
  return kNoError;
}

CubicSpline::SplineError CubicSpline::GetSegmentFunction(
    int index, CubicFunction* segment_function) {
  if (!curve_fitted_) {
    CubicSpline::SplineError error = Fit();
    if (error != kNoError) {
      return error;
    }
  }
  if (index < 0 || index > GetPointCount() - 2) {
    return kIndexOutOfBoundsError;
  }
  *segment_function = segment_functions_[index];
  return kNoError;
}

CubicSpline::SplineError CubicSpline::GetPoint(int index,
                                               Vector2f* point) const {
  if (GetPointCount() < 1) {
    return kNotEnoughPointsError;
  } else if (index < 0 || index >= GetPointCount()) {
    return kIndexOutOfBoundsError;
  } else {
    (*point) = points_[index];
    return kNoError;
  }
}

int CubicSpline::GetSegmentCount() const {
  // Do not use the length of segments_ because that might require an update.
  if (GetPointCount() > 1) {
    return GetPointCount() - 1;
  } else {
    return 0;
  }
}

int CubicSpline::GetPointCount() const {
  return static_cast<int>(points_.size());
}

CubicSpline::SplineError CubicSpline::ClampSlopes(float start_slope,
                                                  float end_slope) {
  if (mode_ != kClamped) {
    return kNotClamped;
  } else {
    start_slope_ = start_slope;
    end_slope_ = end_slope;
    return kNoError;
  }
}

CubicSpline::SplineError CubicSpline::Fit() {
  // The following spline interpolation algorithm is based on the spline section
  // of the following book:
  // Stoer, Josef, Friedrich L. Bauer, and Roland Bulirsch: 'Numerische
  // Mathematik. Vol. 8.', Berlin: Springer-Verlag, 1989.

  // Using the shortcut 'n' for the segment_count in order to simplify the
  // comparison with the above mentioned article.
  const int n = GetSegmentCount();

  if (curve_fitted_) {
    return kNoError;
  } else if (n < 1) {
    return kNotEnoughPointsError;
  }

  // To fit a periodic function, user must make sure that y(0) == y(n).
  if (mode_ == kPeriodic && points_[0](1) != points_[n](1)) {
    return kNotPeriodic;
  }

  int rank = (mode_ == kPeriodic) ? n : n + 1;

  // h contains the x-differences between subsequent points.
  std::vector<float> h(n + 1);
  h[0] = 0.f;  // unused.
  for (int j = 1; j < n + 1; ++j) {
    h[j] = points_[j](0) - points_[j - 1](0);
  }

  // Actually, lambda and mu could both be one element smaller by shifting all
  // indices by one. For consistency with the indices used in the article
  // mentionend above, accept to reserve two floats too much.
  std::vector<float> lambda(n + 1);
  std::vector<float> mu(n + 1);
  std::vector<float> d(n + 1);
  // Following equation 2.4.2.6, fill lambda, mu and d.
  for (int j = 1; j < n; ++j) {
    lambda[j] = h[j + 1] / (h[j] + h[j + 1]);
    mu[j] = 1.f - lambda[j];
    d[j] = 6.f / (h[j] + h[j + 1]) *
           ((points_[j + 1](1) - points_[j](1)) / h[j + 1] -
            (points_[j](1) - points_[j - 1](1)) / h[j]);
  }

  // The endpoints depend on the current mode. (See equations 2.4.2.7, 2.4.2.8
  // and 2.4.2.10.)
  if (mode_ == kNatural) {
    lambda[0] = 0.f;
    d[0] = 0.f;
    mu[n] = 0.f;
    d[n] = 0.f;
  } else if (mode_ == kClamped) {
    lambda[0] = 1.f;
    d[0] = 6.f / h[1] * ((points_[1](1) - points_[0](1)) / h[1] - start_slope_);
    mu[n] = 1.f;
    d[n] =
        6.f / h[n] * (end_slope_ - (points_[n](1) - points_[n - 1](1)) / h[n]);
  } else if (mode_ == kPeriodic) {
    lambda[n] = h[1] / (h[n] + h[1]);
    mu[n] = 1 - lambda[n];
    d[n] = 6.f / (h[n] + h[1]) * ((points_[1](1) - points_[n](1)) / h[1] -
                                  (points_[n](1) - points_[n - 1](1)) / h[n]);
  }

  Eigen::MatrixXf linear_system = Eigen::MatrixXf::Zero(rank, rank);
  Eigen::VectorXf value = Eigen::VectorXf::Zero(rank);
  if (mode_ == kPeriodic) {
    // For the periodic case, solve the following equation system for M.
    // (lambda denoted by l, mu by u)
    // [  2   l1            u1   ]     [ M1 ]     [ d1 ]
    // [  u2   2  l2             ]     [ M2 ]     [ d2 ]
    // [      u3   .  .          ]  *  [ .  ]  =  [ .  ]
    // [           .  .  .       ]     [ .  ]     [ .  ]
    // [              .  2  ln-1 ]     [ .  ]     [ .  ]
    // [  ln             un  2   ]     [ Mn ]     [ dn ]
    for (int j = 0; j < rank; ++j) {
      linear_system(j, j) = 2.f;
      value(j) = d[j + 1];
    }
    for (int j = 0; j < rank - 1; ++j) {
      linear_system(j, j + 1) = lambda[j + 1];
      linear_system(j + 1, j) = mu[j + 2];
    }
    linear_system(0, rank - 1) = mu[1];
    linear_system(rank - 1, 0) = lambda[n];
  } else {
    // For natural or clamped splines, the equation system looks as follows.
    // (lambda denoted by l, mu by u)
    // [  2   l0             0   ]     [ M0 ]     [ d0 ]
    // [  u1   2  l1             ]     [ M1 ]     [ d1 ]
    // [      u2   .  .          ]  *  [ .  ]  =  [ .  ]
    // [           .  .  .       ]     [ .  ]     [ .  ]
    // [              .  2  ln-1 ]     [ .  ]     [ .  ]
    // [  0              un  2   ]     [ Mn ]     [ dn ]
    for (int j = 0; j < rank; ++j) {
      linear_system(j, j) = 2.f;
      value(j) = d[j];
    }
    for (int j = 0; j < rank - 1; ++j) {
      linear_system(j, j + 1) = lambda[j];
      linear_system(j + 1, j) = mu[j + 1];
    }
  }
  // Solve equation system and extract the cubic functions.
  Eigen::VectorXf m = linear_system.colPivHouseholderQr().solve(value);
  segment_functions_.resize(n);
  for (int segment = 0; segment < n; ++segment) {
    CubicFunction& segment_function = segment_functions_[segment];
    // Looking at the equation systems above, it should be noted that in the
    // periodic case, m will not contain element 0. Therefore, the moment
    // index will be shifted backwards by one, modulo rank.
    int index = (mode_ == kPeriodic) ? ((segment + rank - 1) % rank) : segment;
    float m_start = m[index];
    float m_end = m[(index + 1) % rank];

    float difference = points_[segment + 1](1) - points_[segment](1);
    segment_function.set_x_offset(-points_[segment](0));
    segment_function.set_cubic_term((m_end - m_start) / (6.f * h[segment + 1]));
    segment_function.set_quadratic_term(m_start / 2.f);
    segment_function.set_linear_term(difference / h[segment + 1] -
                                     (2.f * m_start + m_end) / 6.f *
                                         h[segment + 1]);
    segment_function.set_absolute_term(points_[segment](1));
  }
  curve_fitted_ = true;
  return kNoError;
}

int CubicSpline::FindSegment(float x) const {
  // Special handling of the simplest case, when x is bigger than the last
  // x-coordinate in the list.
  if (GetPointCount() == 0 || points_[GetPointCount() - 1](0) < x) {
    return GetPointCount() - 1;
  } else if (points_[0](0) > x) {
    // x is smaller than the smallest entry in the list.
    return -1;
  } else {
    // Otherwise, do a binary search to find an insertion point.
    int lower_index = 0;
    int upper_index = GetPointCount();
    while (lower_index < upper_index) {
      int middle_index = (lower_index + upper_index) / 2;
      float middle_value = points_[middle_index](0);
      if (middle_value == x) {
        return middle_index - 1;
      } else if (middle_value < x) {
        lower_index = middle_index + 1;
      } else {
        upper_index = middle_index;
      }
    }

    return lower_index - 1;
  }
}

}  // namespace guideline::util
