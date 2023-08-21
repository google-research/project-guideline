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

// Predicates used for testing.

#ifndef PROJECT_GUIDELINE_TESTING_PREDICATES_H_
#define PROJECT_GUIDELINE_TESTING_PREDICATES_H_

#include "gtest/gtest.h"
#include "Eigen/Core"  // keep include
#include "project_guideline/util/transformation.h"

#define EXPECT_EIGEN_APPROX(a, b, ...) \
  EXPECT_TRUE(guideline::testing::EigenIsApprox(a, b, ##__VA_ARGS__))
#define EXPECT_EIGEN_NOT_APPROX(a, b, ...) \
  EXPECT_FALSE(guideline::testing::EigenIsApprox(a, b, ##__VA_ARGS__))

#define ASSERT_EIGEN_APPROX(a, b, ...) \
  ASSERT_TRUE(guideline::testing::EigenIsApprox(a, b, ##__VA_ARGS__))
#define ASSERT_EIGEN_NOT_APPROX(a, b, ...) \
  ASSERT_FALSE(guideline::testing::EigenIsApprox(a, b, ##__VA_ARGS__))

#define EXPECT_TRANSFORMATION_APPROX(a, b, ...) \
  EXPECT_TRUE(guideline::testing::TransformationIsApprox(a, b, ##__VA_ARGS__))
#define EXPECT_TRANSFORMATION_NOT_APPROX(a, b, ...) \
  EXPECT_FALSE(guideline::testing::TransformationIsApprox(a, b, ##__VA_ARGS__))

#define ASSERT_TRANSFORMATION_APPROX(a, b, ...) \
  ASSERT_TRUE(guideline::testing::TransformationIsApprox(a, b, ##__VA_ARGS__))
#define ASSERT_TRANSFORMATION_NOT_APPROX(a, b, ...) \
  ASSERT_FALSE(guideline::testing::TransformationIsApprox(a, b, ##__VA_ARGS__))

namespace guideline::testing {

template <typename DerivedA, typename DerivedB>
inline ::testing::AssertionResult EigenIsApprox(
    const Eigen::DenseBase<DerivedA>& a, const Eigen::DenseBase<DerivedB>& b,
    double tolerance = Eigen::NumTraits<double>::dummy_precision()) {
  if ((a.derived() - b.derived()).cwiseAbs().maxCoeff() > tolerance) {
    return ::testing::AssertionFailure()
           << "a: " << a << " not within " << tolerance << " of b: " << b;
  }
  return ::testing::AssertionSuccess();
}

inline ::testing::AssertionResult TransformationIsApprox(
    const util::Transformation& a, const util::Transformation& b,
    double tolerance = Eigen::NumTraits<double>::dummy_precision()) {
  if ((a.p() - b.p()).cwiseAbs().maxCoeff() > tolerance ||
      (a.q().coeffs() - b.q().coeffs()).cwiseAbs().maxCoeff() > tolerance) {
    return ::testing::AssertionFailure()
           << "a: " << a << " not within " << tolerance << " of b: " << b;
  }
  return ::testing::AssertionSuccess();
}

}  // namespace guideline::testing

#endif  // PROJECT_GUIDELINE_TESTING_PREDICATES_H_
