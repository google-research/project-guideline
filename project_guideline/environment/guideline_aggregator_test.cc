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

#include "project_guideline/environment/guideline_aggregator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "absl/random/distributions.h"
#include "absl/random/random.h"
#include "project_guideline/proto/guideline_engine_config.pb.h"
#include "project_guideline/testing/predicates.h"
#include "project_guideline/testing/status_matchers.h"
#include "project_guideline/util/geometry.h"
#include "project_guideline/util/hit_test_util.h"
#include "project_guideline/util/transformation.h"

namespace guideline::environment {

namespace {

using ::Eigen::MatrixXd;
using ::Eigen::Vector3d;
using ::Eigen::Vector4d;
using ::Eigen::VectorXd;
using ::testing::DoubleNear;
using ::testing::Pointwise;
using util::Transformation;

using environment_internal::FindClosestPoint;
using environment_internal::PolyEval;
using util::Axis3;
using util::HitResult;

constexpr double kDoubleTolerance = 1e-5;

TEST(GuidelineBoxPoint, AddAndClear) {
  auto box_point = GuidelineBoxPoint(Vector3d(1, 2, 3), 1.0);
  ASSERT_TRUE(box_point.IsKeypointInBox(
      HitResult(Transformation(Vector3d(0.55, 1.55, 2.55)), 0)));
  ASSERT_TRUE(box_point.IsKeypointInBox(
      HitResult(Transformation(Vector3d(1.45, 2.45, 3.45)), 0)));
  ASSERT_FALSE(box_point.IsKeypointInBox(
      HitResult(Transformation(Vector3d(0.49, 1.55, 2.55)), 0)));
  ASSERT_FALSE(box_point.IsKeypointInBox(
      HitResult(Transformation(Vector3d(1.51, 2.45, 3.45)), 0)));
  ASSERT_FALSE(box_point.IsKeypointInBox(
      HitResult(Transformation(Vector3d(1.45, 2.51, 3.45)), 0)));
  ASSERT_FALSE(box_point.IsKeypointInBox(
      HitResult(Transformation(Vector3d(1.45, 2.45, 3.51)), 0)));
  ASSERT_TRUE(box_point.AddKeypointToBox(
      HitResult(Transformation(Vector3d(0.55, 1.55, 2.55)), 0), 0));
  ASSERT_TRUE(box_point.AddKeypointToBox(
      HitResult(Transformation(Vector3d(1.45, 2.45, 3.45)), 0), 0));
  ASSERT_FALSE(box_point.AddKeypointToBox(
      HitResult(Transformation(Vector3d(0.49, 1.55, 2.55)), 0), 0));
  ASSERT_FALSE(box_point.AddKeypointToBox(
      HitResult(Transformation(Vector3d(1.51, 2.45, 3.45)), 0), 0));
  ASSERT_FALSE(box_point.AddKeypointToBox(
      HitResult(Transformation(Vector3d(1.45, 2.51, 3.45)), 0), 0));
  ASSERT_FALSE(box_point.AddKeypointToBox(
      HitResult(Transformation(Vector3d(1.45, 2.45, 3.51)), 0), 0));

  ASSERT_EQ(box_point.timestamp_keypoints.size(), 1);
  ASSERT_EQ(box_point.timestamp_keypoints.find(0)->second.size(), 2);
  ASSERT_EQ(box_point.timestamp_keypoints.find(1),
            box_point.timestamp_keypoints.end());

  ASSERT_TRUE(box_point.AddKeypointToBox(
      HitResult(Transformation(Vector3d(0.55, 1.55, 2.55)), 0), 1));
  ASSERT_TRUE(box_point.AddKeypointToBox(
      HitResult(Transformation(Vector3d(1.45, 2.45, 3.45)), 0), 1));
  std::vector<int64_t> timestamps = {0};
  box_point.ClearEntries(timestamps);
  ASSERT_EQ(box_point.timestamp_keypoints.find(0),
            box_point.timestamp_keypoints.end());
  ASSERT_EQ(box_point.timestamp_keypoints.size(), 1);
  ASSERT_EQ(box_point.timestamp_keypoints.find(1)->second.size(), 2);
  timestamps = {0, 1};
  box_point.ClearEntries(timestamps);
  ASSERT_EQ(box_point.timestamp_keypoints.find(0),
            box_point.timestamp_keypoints.end());
  ASSERT_EQ(box_point.timestamp_keypoints.find(1),
            box_point.timestamp_keypoints.end());
  ASSERT_EQ(box_point.timestamp_keypoints.size(), 0);
}

TEST(GuidelineBoxPoint, Confidence) {
  auto box_point = GuidelineBoxPoint(Vector3d(1, 2, 3), 1);
  ASSERT_FLOAT_EQ(box_point.CountAndConfidence().first, 0);
  ASSERT_FLOAT_EQ(box_point.CountAndConfidence().second, 0);

  box_point.AddKeypointToBox(
      HitResult(Transformation(Vector3d(0.55, 1.55, 2.55)), 0, 0.7), 0);
  box_point.AddKeypointToBox(
      HitResult(Transformation(Vector3d(1.45, 2.45, 3.45)), 0, 0.7), 0);
  ASSERT_FLOAT_EQ(box_point.CountAndConfidence().first, 2);
  ASSERT_FLOAT_EQ(box_point.CountAndConfidence().second, 0.7);
  box_point.AddKeypointToBox(
      HitResult(Transformation(Vector3d(0.55, 1.55, 2.55)), 0, 0.9), 1);
  box_point.AddKeypointToBox(
      HitResult(Transformation(Vector3d(1.45, 2.45, 3.45)), 0, 0.9), 1);
  ASSERT_FLOAT_EQ(box_point.CountAndConfidence().first, 4);
  ASSERT_FLOAT_EQ(box_point.CountAndConfidence().second, 0.8);
  std::vector<int64_t> timestamps = {0};
  box_point.ClearEntries(timestamps);
  ASSERT_FLOAT_EQ(box_point.CountAndConfidence().first, 2);
  ASSERT_FLOAT_EQ(box_point.CountAndConfidence().second, 0.9);
  timestamps = {0, 1};
  box_point.ClearEntries(timestamps);
  ASSERT_FLOAT_EQ(box_point.CountAndConfidence().first, 0);
  ASSERT_FLOAT_EQ(box_point.CountAndConfidence().second, 0);
}

SortedGuidelineBoxPointAggregatorOptions
GetDefaultSortedGuidelineBoxPointAggregatorOptions() {
  SortedGuidelineBoxPointAggregatorOptions options;
  options.set_box_size_meters(1);
  options.set_confidence_threshold(0.5);
  options.set_num_points_threshold(1);
  options.set_num_meters_per_spline(2);
  options.set_forward_extrapolation_meters(5);
  options.set_backward_extrapolation_meters(5);
  options.set_num_guideline_points_per_meter(1);
  return options;
}

LocalTemporalRegressionBasedGuidelineAggregatorOptions
GetDefaultLocalTemporalRegressionBasedGuidelineAggregatorOptions() {
  LocalTemporalRegressionBasedGuidelineAggregatorOptions options;
  options.set_num_guideline_points_per_meter(1);
  options.set_forward_extrapolation_meters(5);
  options.set_backward_extrapolation_meters(5);
  return options;
}

std::unique_ptr<GuidelineAggregator> SGBATestInitVariables(
    std::unique_ptr<GuidelineAggregator> guideline_aggregator,
    std::vector<Vector3d>& expected_centers, std::vector<float>& xs,
    std::vector<float>& ys, std::vector<float>& expected_xs,
    std::vector<float>& expected_ys, Axis3 vertical_axis) {
  std::vector<HitResult> ordered_hit_results;
  for (int i = 0; i < xs.size(); ++i) {
    ordered_hit_results.push_back(
        HitResult(Transformation(Vector3d(xs.at(i), ys.at(i), 0)), 0, 0.9));
  }
  guideline_aggregator->AggregateKeypoints(ordered_hit_results, 0,
                                           vertical_axis);
  for (int i = 0; i < expected_xs.size(); ++i) {
    expected_centers.push_back(
        Vector3d(expected_xs.at(i), expected_ys.at(i), 0));
  }
  return guideline_aggregator;
}

TEST(SortedGuidelineBoxPointAggregatorTest, TestAddOrderedKeypoints) {
  std::vector<float> xs, ys, expected_xs, expected_ys;
  xs = {-12, -6., -5., -5., -4., -3., -3., -2., -2., -1., -0.,
        0.,  1.,  1.,  2.,  2.,  3.,  3.,  4.,  5.,  5.,  6.};
  ys = {-2,  0.,   0., 0., 1., 3., 3., 6., 6., 9., 10.,
        10., 11.0, 9., 6., 6., 3., 3., 1., 0., 0., 0.};
  expected_xs = {-12, -6., -5., -4., -3., -2., -1., -0.,
                 1.,  1.,  2.,  3.,  4.,  5.,  6.};
  expected_ys = {-2, 0., 0., 1., 3., 6., 9., 10., 11.0, 9., 6., 3., 1., 0., 0.};
  auto main_options = GuidelineAggregatorOptions();
  auto options =
      main_options.mutable_sorted_guideline_box_point_aggregator_options();
  *options = GetDefaultSortedGuidelineBoxPointAggregatorOptions();
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator1,
      SortedGuidelineBoxPointAggregator::Create(main_options));
  std::vector<Vector3d> expected_centers1;
  guideline_aggregator1 =
      SGBATestInitVariables(std::move(guideline_aggregator1), expected_centers1,
                            xs, ys, expected_xs, expected_ys, Axis3::kZ);
  SortedGuidelineBoxPointAggregator* derived_ptr1 =
      dynamic_cast<SortedGuidelineBoxPointAggregator*>(
          guideline_aggregator1.get());
  auto sorted_box_points = derived_ptr1->GetSortedBoxPoints();
  ASSERT_EQ(sorted_box_points.size(), expected_centers1.size());
  for (int i = 0; i < expected_centers1.size(); ++i) {
    ASSERT_EQ(sorted_box_points.at(i).center, expected_centers1.at(i));
  }
  ASSERT_NEAR(sorted_box_points.back().distance_to_anchor, 30.76, 1e-02);

  xs = {-12, -6., -5., -5., -4., -3., -3., -2., -2., -1., -0.,
        0.,  1.,  1.,  2.,  2.,  3.,  3.,  4.,  5.,  5.,  6.};
  ys = {-2,  0.,   0., 0., 1., 3., 3., 6., 6., 9., 10.,
        10., 11.0, 9., 6., 6., 3., 3., 1., 0., 0., 0.};
  expected_xs = {-12, -6., -5., -4., -3., -2., -1., -0.,
                 1.,  1.,  2.,  3.,  4.,  5.,  6.};
  expected_ys = {-2, 0., 0., 1., 3., 6., 9., 10., 11.0, 9., 6., 3., 1., 0., 0.};
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator2,
      SortedGuidelineBoxPointAggregator::Create(main_options));
  std::vector<Vector3d> expected_centers2;
  guideline_aggregator2 =
      SGBATestInitVariables(std::move(guideline_aggregator2), expected_centers2,
                            xs, ys, expected_xs, expected_ys, Axis3::kZ);
  auto derived_ptr2 = dynamic_cast<SortedGuidelineBoxPointAggregator*>(
      guideline_aggregator2.get());
  sorted_box_points = derived_ptr2->GetSortedBoxPoints();
  ASSERT_EQ(sorted_box_points.size(), expected_centers2.size());
  for (int i = 0; i < expected_centers2.size(); ++i) {
    ASSERT_EQ(sorted_box_points.at(i).center, expected_centers2.at(i));
  }
  ASSERT_NEAR(sorted_box_points.back().distance_to_anchor, 30.76, 1e-02);

  xs = {-12, -6., -5., -5., -4., -3., -3., -2., -2., -1., -0.,
        0.,  1.,  1.,  2.,  2.,  3.,  3.,  4.,  5.,  5.,  6.};
  ys = {-2,  0., 0.,   0., 1., 3., 3., 6., 6., 9., 10.,
        10., 9., 11.0, 6., 6., 3., 3., 1., 0., 0., 0.};
  expected_xs = {-12, -6., -5., -4., -3., -2., -1., -0.,
                 1.,  1.,  2.,  3.,  4.,  5.,  6.};
  expected_ys = {-2, 0., 0., 1., 3., 6., 9., 10., 9., 11.0, 6., 3., 1., 0., 0.};
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator3,
      SortedGuidelineBoxPointAggregator::Create(main_options));
  std::vector<Vector3d> expected_centers3;
  guideline_aggregator3 =
      SGBATestInitVariables(std::move(guideline_aggregator3), expected_centers3,
                            xs, ys, expected_xs, expected_ys, Axis3::kZ);
  auto derived_ptr3 = dynamic_cast<SortedGuidelineBoxPointAggregator*>(
      guideline_aggregator3.get());
  sorted_box_points = derived_ptr3->GetSortedBoxPoints();
  ASSERT_EQ(sorted_box_points.size(), expected_centers3.size());
  for (int i = 0; i < expected_centers3.size(); ++i) {
    ASSERT_EQ(sorted_box_points.at(i).center, expected_centers3.at(i));
  }
  ASSERT_NEAR(sorted_box_points.back().distance_to_anchor, 30.76, 1e-02);
}

TEST(SortedGuidelineBoxPointAggregatorTest, TestGetGuideline_1) {
  auto main_options = GuidelineAggregatorOptions();
  auto options =
      main_options.mutable_sorted_guideline_box_point_aggregator_options();
  *options = GetDefaultSortedGuidelineBoxPointAggregatorOptions();
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator0,
      SortedGuidelineBoxPointAggregator::Create(main_options));
  ASSERT_FALSE(guideline_aggregator0->GetGuideline().ok());
  std::vector<HitResult> hit_results;
  for (float i = 0; i <= 10; i++) {
    hit_results.push_back(HitResult(Transformation(Vector3d(i, 0, 0)), 0, 0.9));
  }
  guideline_aggregator0->AggregateKeypoints(hit_results, 0, Axis3::kZ);
  GL_ASSERT_OK_AND_ASSIGN(auto guideline_points0,
                          guideline_aggregator0->GetGuideline());
  ASSERT_EQ(guideline_points0.size(), 20);
  EXPECT_EIGEN_APPROX(guideline_points0.front(), Vector3d(-5, 0, 0));
  EXPECT_EIGEN_APPROX(guideline_points0.back(), Vector3d(14, 0, 0));

  options->set_num_guideline_points_per_meter(2);
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator1,
      SortedGuidelineBoxPointAggregator::Create(main_options));
  hit_results.clear();
  for (float i = 0; i <= 10; i++) {
    hit_results.push_back(HitResult(Transformation(Vector3d(i, 0, 0)), 0, 0.9));
  }
  guideline_aggregator1->AggregateKeypoints(hit_results, 1, Axis3::kZ);
  GL_ASSERT_OK_AND_ASSIGN(auto guideline_points1,
                          guideline_aggregator1->GetGuideline());
  ASSERT_EQ(guideline_points1.size(), 40);
  EXPECT_EIGEN_APPROX(guideline_points1.front(), Vector3d(-5, 0, 0),
                      kDoubleTolerance);
  EXPECT_EIGEN_APPROX(guideline_points1.back(), Vector3d(14.5, 0, 0),
                      kDoubleTolerance);
  auto derived_ptr1 = dynamic_cast<SortedGuidelineBoxPointAggregator*>(
      guideline_aggregator1.get());
  ASSERT_NEAR(derived_ptr1->GetSortedBoxPoints().back().distance_to_anchor, 10,
              1e-03);

  options->set_confidence_threshold(0.95);
  options->set_num_points_threshold(2);
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator2,
      SortedGuidelineBoxPointAggregator::Create(main_options));
  hit_results.clear();
  for (float i = 0; i <= 10; i++) {
    hit_results.push_back(HitResult(Transformation(Vector3d(i, 0, 0)), 0, 0.9));
  }
  guideline_aggregator2->AggregateKeypoints(hit_results, 2, Axis3::kZ);
  ASSERT_FALSE(guideline_aggregator2->GetGuideline().ok());
}

TEST(SortedGuidelineBoxPointAggregatorDeathTest, TestGetGuidelineDistanceZero) {
  auto main_options = GuidelineAggregatorOptions();
  auto options =
      main_options.mutable_sorted_guideline_box_point_aggregator_options();
  *options = GetDefaultSortedGuidelineBoxPointAggregatorOptions();
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator,
      SortedGuidelineBoxPointAggregator::Create(main_options));
  std::vector<HitResult> hit_results;
  for (float i = 0; i <= 10; i++) {
    hit_results.push_back(HitResult(Transformation(Vector3d(i, 0, 0)), 0, 0.9));
  }
  ASSERT_DEATH(
      guideline_aggregator->AggregateKeypoints(hit_results, 0, Axis3::kX), "");
  ASSERT_FALSE(guideline_aggregator->GetGuideline().ok());
}

TEST(SortedGuidelineBoxPointAggregatorTest, TestGetGuideline_2) {
  std::vector<float> xs, ys, expected_xs, expected_ys;
  xs = {-12, -6., -5., -5., -4., -3., -3., -2., -2., -1., -0.,
        0.,  1.,  1.,  2.,  2.,  3.,  3.,  4.,  5.,  5.,  6.};
  ys = {-2,  0.,   0., 0., 1., 3., 3., 6., 6., 9., 10.,
        10., 11.0, 9., 6., 6., 3., 3., 1., 0., 0., 0.};
  expected_xs = {-12, -6., -5., -4., -3., -2., -1., -0.,
                 1.,  1.,  2.,  3.,  4.,  5.,  6.};
  expected_ys = {-2, 0., 0., 1., 3., 6., 9., 10., 11.0, 9., 6., 3., 1., 0., 0.};
  auto main_options = GuidelineAggregatorOptions();
  auto options =
      main_options.mutable_sorted_guideline_box_point_aggregator_options();
  *options = GetDefaultSortedGuidelineBoxPointAggregatorOptions();
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator,
      SortedGuidelineBoxPointAggregator::Create(main_options));
  std::vector<Vector3d> expected_centers;
  guideline_aggregator =
      SGBATestInitVariables(std::move(guideline_aggregator), expected_centers,
                            xs, ys, expected_xs, expected_ys, Axis3::kZ);
  GL_ASSERT_OK_AND_ASSIGN(auto guideline_points,
                          guideline_aggregator->GetGuideline());
  EXPECT_EIGEN_APPROX(guideline_points.front(),
                      Vector3d(-16.743416, -3.5811388, 0), kDoubleTolerance);
  EXPECT_EIGEN_APPROX(guideline_points.at(1), Vector3d(-15.7767, -3.2589, 0),
                      kDoubleTolerance);
  EXPECT_EIGEN_APPROX(guideline_points.back(), Vector3d(10.22033, -2.110165, 0),
                      kDoubleTolerance);
}

TEST(SortedGuidelineBoxPointAggregatorTest, TestClearEntries) {
  std::vector<float> xs, ys, expected_xs, expected_ys;
  xs = {-12, -6., -5., -5., -4., -3., -3., -2., -2., -1., -0.,
        0.,  1.,  1.,  2.,  2.,  3.,  3.,  4.,  5.,  5.,  6.};
  ys = {-2,  0.,   0., 0., 1., 3., 3., 6., 6., 9., 10.,
        10., 11.0, 9., 6., 6., 3., 3., 1., 0., 0., 0.};
  expected_xs = {-12, -6., -5., -4., -3., -2., -1., -0.,
                 1.,  1.,  2.,  3.,  4.,  5.,  6.};
  expected_ys = {-2, 0., 0., 1., 3., 6., 9., 10., 11.0, 9., 6., 3., 1., 0., 0.};
  auto main_options = GuidelineAggregatorOptions();
  auto options =
      main_options.mutable_sorted_guideline_box_point_aggregator_options();
  *options = GetDefaultSortedGuidelineBoxPointAggregatorOptions();
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator,
      SortedGuidelineBoxPointAggregator::Create(main_options));
  std::vector<Vector3d> expected_centers;
  for (int i = 0; i < xs.size(); ++i) {
    auto hit_result =
        HitResult(Transformation(Vector3d(xs.at(i), ys.at(i), 0)), 0, 0.9);
    guideline_aggregator->AggregateKeypoints({hit_result}, i, Axis3::kZ);
  }

  for (int i = 0; i < expected_xs.size(); ++i) {
    expected_centers.push_back(
        Vector3d(expected_xs.at(i), expected_ys.at(i), 0));
  }
  auto derived_ptr = dynamic_cast<SortedGuidelineBoxPointAggregator*>(
      guideline_aggregator.get());
  auto sorted_box_points = derived_ptr->GetSortedBoxPoints();
  ASSERT_EQ(sorted_box_points.size(), expected_centers.size());
  for (int i = 0; i < expected_xs.size(); ++i) {
    ASSERT_EQ(sorted_box_points.at(i).center, expected_centers.at(i));
  }
  ASSERT_NEAR(sorted_box_points.front().distance_to_anchor, 0, 1e-04);
  ASSERT_NEAR(sorted_box_points.back().distance_to_anchor, 30.76, 0.5);

  std::vector<int64_t> timestamps = {0, 1, 2, 4, 3};
  GL_ASSERT_OK(guideline_aggregator->ClearEntries(5));
  expected_centers.clear();
  for (int i = 4; i < expected_xs.size(); ++i) {
    expected_centers.push_back(
        Vector3d(expected_xs.at(i), expected_ys.at(i), 0));
  }
  sorted_box_points = derived_ptr->GetSortedBoxPoints();
  ASSERT_EQ(sorted_box_points.size(), expected_centers.size());
  for (int i = 0; i < expected_centers.size(); ++i) {
    ASSERT_EQ(sorted_box_points.at(i).center, expected_centers.at(i));
  }
  ASSERT_NEAR(sorted_box_points.front().distance_to_anchor, 0, 1e-04);
  ASSERT_NEAR(sorted_box_points.back().distance_to_anchor, 20.12, 0.5);
  GL_ASSERT_OK(guideline_aggregator->ClearEntries(std::nullopt));
  ASSERT_EQ(derived_ptr->GetSortedBoxPoints().size(), 0);
  timestamps.clear();
  ASSERT_FALSE(guideline_aggregator->ClearEntries(5).ok());
  ASSERT_EQ(derived_ptr->GetSortedBoxPoints().size(), 0);
}

TEST(LocalTemporalRegressionBasedGuidelineAggregatorTest, TestPolyEval) {
  VectorXd coeffs(4);
  coeffs(0) = 1;
  coeffs(1) = 2;
  coeffs(2) = 3;
  coeffs(3) = 4;
  ASSERT_EQ(PolyEval(coeffs, 1), 10);
  ASSERT_EQ(PolyEval(coeffs, 2), 49);
  ASSERT_EQ(PolyEval(coeffs, 3), 142);
}

TEST(LocalTemporalRegressionBasedGuidelineAggregatorTest,
     TestFindClosestPoint) {
  std::vector<Vector3d> points;
  for (int i = 0; i < 10; ++i) {
    points.push_back(Vector3d(i, i, i));
  }
  Vector3d target_point(7.7, 7.7, 7.7);
  std::pair<size_t, double> output = FindClosestPoint(points, 0, target_point);
  ASSERT_EQ(output.first, 8);
  ASSERT_FLOAT_EQ(output.second, std::sqrt(3 * 0.3 * 0.3));
}

TEST(LocalTemporalRegressionBasedGuidelineAggregatorTest, TestFitCurve) {
  GuidelineAggregatorOptions main_options;
  auto* options =
      main_options
          .mutable_local_temporal_regression_based_guideline_aggregator_options();  // NOLINT(whitespace/line_length)
  *options = GetDefaultLocalTemporalRegressionBasedGuidelineAggregatorOptions();
  options->set_num_guideline_points_per_meter(2);
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator,
      LocalTemporalRegressionBasedGuidelineAggregator::Create(main_options));
  ASSERT_FALSE(guideline_aggregator->GetGuideline().ok());
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;

  // Straight line.
  for (double i = 0; i < 10; ++i) {
    xs.push_back(i);
    ys.push_back(i - 0.2);
    zs.push_back(i - 0.2);
    xs.push_back(i);
    ys.push_back(i + 0.2);
    zs.push_back(i + 0.2);
  }
  std::vector<double> expected_fitted_zs = {0,   0.5, 1,   1.5, 2,   2.5, 3,
                                            3.5, 4,   4.5, 5,   5.5, 6,   6.5,
                                            7,   7.5, 8,   8.5, 9};
  std::vector<double> expected_fitted_xs = {0,   0.5, 1,   1.5, 2,   2.5, 3,
                                            3.5, 4,   4.5, 5,   5.5, 6,   6.5,
                                            7,   7.5, 8,   8.5, 9};

  auto derived_ptr =
      dynamic_cast<LocalTemporalRegressionBasedGuidelineAggregator*>(
          guideline_aggregator.get());
  FitCurveOutput fitted_values;

  GL_ASSERT_OK_AND_ASSIGN(fitted_values, derived_ptr->FitCurve(xs, ys, zs, 1));
  EXPECT_THAT(fitted_values.independent_variable_values,
              Pointwise(DoubleNear(0.001), expected_fitted_xs));
  EXPECT_THAT(fitted_values.dependent_variable1_values,
              Pointwise(DoubleNear(0.001), expected_fitted_zs));
  EXPECT_THAT(fitted_values.dependent_variable2_values,
              Pointwise(DoubleNear(0.001), expected_fitted_zs));

  // Quadratic curve.
  xs.clear();
  ys.clear();
  zs.clear();
  xs = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  ys = {0, 1, 4, 9, 16, 25, 36, 49, 64, 81};
  zs = {0, 1, 4, 9, 16, 25, 36, 49, 64, 81};
  GL_ASSERT_OK_AND_ASSIGN(fitted_values, derived_ptr->FitCurve(xs, ys, zs, 1));

  expected_fitted_zs = {-12, -7.5, -3, 1.5,  6,  10.5, 15, 19.5, 24, 28.5,
                        33,  37.5, 42, 46.5, 51, 55.5, 60, 64.5, 69};
  EXPECT_THAT(fitted_values.independent_variable_values,
              Pointwise(DoubleNear(0.001), expected_fitted_xs));
  EXPECT_THAT(fitted_values.dependent_variable1_values,
              Pointwise(DoubleNear(0.001), expected_fitted_zs));
  EXPECT_THAT(fitted_values.dependent_variable2_values,
              Pointwise(DoubleNear(0.001), expected_fitted_zs));
}

std::vector<std::vector<HitResult>> GetStraightNoisyGuideline(
    bool negative_coordinates = false) {
  float kInternalShift = 4;
  float kInternalSize = 16;
  float kDistance = 160;
  std::vector<HitResult> gt_complete_guideline;
  float t = 0;
  while (t < kDistance) {
    if (negative_coordinates) {
      gt_complete_guideline.push_back(
          HitResult(Transformation(Vector3d(0, -t, 0)), 0, 0.9));
    } else {
      gt_complete_guideline.push_back(
          HitResult(Transformation(Vector3d(0, t, 0)), 0, 0.9));
    }
    t += 1;
  }

  std::vector<std::vector<HitResult>> timestamp_ordered_gt_keypoints;
  for (size_t i = 0; i < gt_complete_guideline.size(); i = i + kInternalShift) {
    timestamp_ordered_gt_keypoints.push_back(std::vector<HitResult>());
    for (size_t j = 0; j < kInternalSize; ++j) {
      if (i + j >= gt_complete_guideline.size()) {
        break;
      }
      timestamp_ordered_gt_keypoints.back().push_back(
          gt_complete_guideline.at(i + j));
    }
  }

  std::seed_seq my_seed_seq({1, 2, 3});
  absl::BitGen bitgen(my_seed_seq);
  float noise_level_x = 4;
  float noise_level_y = 2;
  float noise_x;
  float noise_y;
  float x, y;
  std::vector<std::vector<HitResult>> timestamp_ordered_noisy_keypoints;
  for (size_t t = 0; t < timestamp_ordered_gt_keypoints.size(); ++t) {
    timestamp_ordered_noisy_keypoints.push_back(std::vector<HitResult>());
    for (size_t i = 0; i < timestamp_ordered_gt_keypoints.at(t).size(); ++i) {
      noise_x = absl::Uniform(bitgen, -noise_level_x, noise_level_x);
      noise_y = absl::Uniform(bitgen, -noise_level_y, noise_level_y);
      // Make first and last point non noisy.
      if (t == 0 || t == timestamp_ordered_gt_keypoints.size() - 1) {
        noise_x = 0;
        noise_y = 0;
      }
      x = timestamp_ordered_gt_keypoints.at(t).at(i).hit_pose.p().x() + noise_x;
      y = timestamp_ordered_gt_keypoints.at(t).at(i).hit_pose.p().y() + noise_y;
      timestamp_ordered_noisy_keypoints.back().push_back(
          HitResult(Transformation(Vector3d(x, y, 0)), 0, 0.9));
    }
  }
  return timestamp_ordered_noisy_keypoints;
}

std::vector<std::vector<HitResult>> GetEllipticalGuideline() {
  float kInternalShift = 4;
  float kInternalSize = 16;
  float kRadius = 30;
  std::vector<HitResult> gt_complete_guideline;
  float t = 0;
  float x, y;
  while (t < 2 * M_PI) {
    x = kRadius * std::cos(t);
    y = kRadius * std::sin(t);
    gt_complete_guideline.push_back(
        HitResult(Transformation(Vector3d(x, y, 0)), 0, 0.9));
    t += 0.04;
  }

  std::vector<std::vector<HitResult>> timestamp_ordered_gt_keypoints;
  for (size_t i = 0; i < gt_complete_guideline.size(); i = i + kInternalShift) {
    timestamp_ordered_gt_keypoints.push_back(std::vector<HitResult>());
    for (size_t j = 0; j < kInternalSize; ++j) {
      if (i + j >= gt_complete_guideline.size()) {
        break;
      }
      timestamp_ordered_gt_keypoints.back().push_back(
          gt_complete_guideline.at(i + j));
    }
  }

  return timestamp_ordered_gt_keypoints;
}

TEST(LocalTemporalRegressionBasedGuidelineAggregator,
     TestGetGuidelineStraight) {
  GuidelineAggregatorOptions main_options;
  auto* options =
      main_options
          .mutable_local_temporal_regression_based_guideline_aggregator_options();  // NOLINT(whitespace/line_length)
  *options = GetDefaultLocalTemporalRegressionBasedGuidelineAggregatorOptions();
  options->set_time_interval_size(10);
  options->set_time_interval_overlap(2);
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator,
      LocalTemporalRegressionBasedGuidelineAggregator::Create(main_options));
  auto timestamp_ordered_noisy_keypoints = GetStraightNoisyGuideline();

  guideline_aggregator->AggregateKeypoints(
      timestamp_ordered_noisy_keypoints.at(0), 0, Axis3::kZ);
  GL_ASSERT_OK_AND_ASSIGN(auto guideline, guideline_aggregator->GetGuideline());

  std::vector<float> fitted_xs;
  std::vector<float> fitted_ys;
  std::vector<float> fitted_zs;
  for (const auto& point : guideline) {
    fitted_xs.push_back(point.x());
    fitted_ys.push_back(point.y());
    fitted_zs.push_back(point.z());
  }

  ASSERT_NEAR(fitted_xs.front(), 0, 2);
  ASSERT_NEAR(fitted_xs.back(), 0, 2);
  auto min_ys_indx =
      std::min_element(fitted_ys.begin(), fitted_ys.end()) - fitted_ys.begin();
  auto max_ys_indx =
      std::max_element(fitted_ys.begin(), fitted_ys.end()) - fitted_ys.begin();
  ASSERT_LE(fitted_ys[min_ys_indx], -4);
  ASSERT_GE(fitted_ys[max_ys_indx], 17);
  ASSERT_NEAR(fitted_zs.front(), 0, 0.001);

  for (size_t t = 1; t < timestamp_ordered_noisy_keypoints.size(); ++t) {
    guideline_aggregator->AggregateKeypoints(
        timestamp_ordered_noisy_keypoints.at(t), t, Axis3::kZ);
  }
  GL_ASSERT_OK_AND_ASSIGN(auto guideline1,
                          guideline_aggregator->GetGuideline());
  fitted_xs.clear();
  fitted_ys.clear();
  fitted_zs.clear();
  for (const auto& point : guideline1) {
    fitted_xs.push_back(point.x());
    fitted_ys.push_back(point.y());
    fitted_zs.push_back(point.z());
  }
  ASSERT_NEAR(fitted_xs.front(), 0, 2);
  ASSERT_NEAR(fitted_xs.back(), 0, 2.5);
  min_ys_indx =
      std::min_element(fitted_ys.begin(), fitted_ys.end()) - fitted_ys.begin();
  max_ys_indx =
      std::max_element(fitted_ys.begin(), fitted_ys.end()) - fitted_ys.begin();
  ASSERT_LE(fitted_ys[min_ys_indx], -4);
  ASSERT_GE(fitted_ys[max_ys_indx], 161);
  ASSERT_NEAR(fitted_zs.front(), 0, 0.001);
}

TEST(LocalTemporalRegressionBasedGuidelineAggregator,
     TestGetGuidelineStraightFittedCurveExtrapolation) {
  GuidelineAggregatorOptions main_options;
  auto* options =
      main_options
          .mutable_local_temporal_regression_based_guideline_aggregator_options();  // NOLINT(whitespace/line_length)
  *options = GetDefaultLocalTemporalRegressionBasedGuidelineAggregatorOptions();
  options->set_time_interval_size(10);
  options->set_time_interval_overlap(2);
  options->set_use_fitted_curve_for_extrapolation(true);
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator,
      LocalTemporalRegressionBasedGuidelineAggregator::Create(main_options));
  auto timestamp_ordered_noisy_keypoints = GetStraightNoisyGuideline();

  guideline_aggregator->AggregateKeypoints(
      timestamp_ordered_noisy_keypoints.at(0), 0, Axis3::kZ);
  GL_ASSERT_OK_AND_ASSIGN(auto guideline, guideline_aggregator->GetGuideline());

  std::vector<float> fitted_xs;
  std::vector<float> fitted_ys;
  std::vector<float> fitted_zs;
  for (const auto& point : guideline) {
    fitted_xs.push_back(point.x());
    fitted_ys.push_back(point.y());
    fitted_zs.push_back(point.z());
  }

  ASSERT_NEAR(fitted_xs.front(), 0, 2);
  ASSERT_NEAR(fitted_xs.back(), 0, 2);
  auto min_ys_indx =
      std::min_element(fitted_ys.begin(), fitted_ys.end()) - fitted_ys.begin();
  auto max_ys_indx =
      std::max_element(fitted_ys.begin(), fitted_ys.end()) - fitted_ys.begin();
  ASSERT_LE(fitted_ys[min_ys_indx], -4);
  ASSERT_GE(fitted_ys[max_ys_indx], 17);
  ASSERT_NEAR(fitted_zs.front(), 0, 0.001);

  for (size_t t = 1; t < timestamp_ordered_noisy_keypoints.size(); ++t) {
    guideline_aggregator->AggregateKeypoints(
        timestamp_ordered_noisy_keypoints.at(t), t, Axis3::kZ);
  }
  GL_ASSERT_OK_AND_ASSIGN(auto guideline1,
                          guideline_aggregator->GetGuideline());
  fitted_xs.clear();
  fitted_ys.clear();
  fitted_zs.clear();
  for (const auto& point : guideline1) {
    fitted_xs.push_back(point.x());
    fitted_ys.push_back(point.y());
    fitted_zs.push_back(point.z());
  }
  ASSERT_NEAR(fitted_xs.front(), 0, 2);
  ASSERT_NEAR(fitted_xs.back(), 0, 3.5);
  min_ys_indx =
      std::min_element(fitted_ys.begin(), fitted_ys.end()) - fitted_ys.begin();
  max_ys_indx =
      std::max_element(fitted_ys.begin(), fitted_ys.end()) - fitted_ys.begin();
  ASSERT_LE(fitted_ys[min_ys_indx], -4);
  ASSERT_GE(fitted_ys[max_ys_indx], 161);
  ASSERT_NEAR(fitted_zs.front(), 0, 0.001);
}

TEST(LocalTemporalRegressionBasedGuidelineAggregator,
     TestGetGuidelineStraightFittedCurveExtrapolationNegativeCoordinates) {
  GuidelineAggregatorOptions main_options;
  auto* options =
      main_options
          .mutable_local_temporal_regression_based_guideline_aggregator_options();  // NOLINT(whitespace/line_length)
  *options = GetDefaultLocalTemporalRegressionBasedGuidelineAggregatorOptions();
  options->set_time_interval_size(10);
  options->set_time_interval_overlap(2);
  options->set_use_fitted_curve_for_extrapolation(true);
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator,
      LocalTemporalRegressionBasedGuidelineAggregator::Create(main_options));
  auto timestamp_ordered_noisy_keypoints = GetStraightNoisyGuideline(true);

  guideline_aggregator->AggregateKeypoints(
      timestamp_ordered_noisy_keypoints.at(0), 0, Axis3::kZ);
  GL_ASSERT_OK_AND_ASSIGN(auto guideline, guideline_aggregator->GetGuideline());

  std::vector<float> fitted_xs;
  std::vector<float> fitted_ys;
  std::vector<float> fitted_zs;
  for (const auto& point : guideline) {
    fitted_xs.push_back(point.x());
    fitted_ys.push_back(point.y());
    fitted_zs.push_back(point.z());
  }
  ASSERT_EQ(fitted_xs.size(), 25);
  ASSERT_NEAR(fitted_xs.front(), 0, 2);
  ASSERT_NEAR(fitted_xs.back(), 0, 2);
  auto min_ys_indx =
      std::min_element(fitted_ys.begin(), fitted_ys.end()) - fitted_ys.begin();
  auto max_ys_indx =
      std::max_element(fitted_ys.begin(), fitted_ys.end()) - fitted_ys.begin();
  ASSERT_LE(fitted_ys[min_ys_indx], -17);
  ASSERT_GE(fitted_ys[max_ys_indx], 5);
  ASSERT_NEAR(fitted_zs.front(), 0, 0.001);

  for (size_t t = 1; t < timestamp_ordered_noisy_keypoints.size(); ++t) {
    guideline_aggregator->AggregateKeypoints(
        timestamp_ordered_noisy_keypoints.at(t), t, Axis3::kZ);
  }
  GL_ASSERT_OK_AND_ASSIGN(auto guideline1,
                          guideline_aggregator->GetGuideline());
  fitted_xs.clear();
  fitted_ys.clear();
  fitted_zs.clear();
  for (const auto& point : guideline1) {
    fitted_xs.push_back(point.x());
    fitted_ys.push_back(point.y());
    fitted_zs.push_back(point.z());
  }
  ASSERT_GT(fitted_xs.size(), 160);
  ASSERT_NEAR(fitted_xs.front(), 0, 2.5);
  ASSERT_NEAR(fitted_xs.back(), 0, 3.5);
  min_ys_indx =
      std::min_element(fitted_ys.begin(), fitted_ys.end()) - fitted_ys.begin();
  max_ys_indx =
      std::max_element(fitted_ys.begin(), fitted_ys.end()) - fitted_ys.begin();
  ASSERT_LE(fitted_ys[min_ys_indx], -161);
  ASSERT_GE(fitted_ys[max_ys_indx], 4);
  ASSERT_NEAR(fitted_zs.front(), 0, 0.001);
}

TEST(LocalTemporalRegressionBasedGuidelineAggregator,
     TestGetGuidelineElliptical) {
  GuidelineAggregatorOptions main_options;
  auto* options =
      main_options
          .mutable_local_temporal_regression_based_guideline_aggregator_options();  // NOLINT(whitespace/line_length)
  *options = GetDefaultLocalTemporalRegressionBasedGuidelineAggregatorOptions();
  options->set_time_interval_size(10);
  options->set_time_interval_overlap(2);
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator,
      LocalTemporalRegressionBasedGuidelineAggregator::Create(main_options));
  ASSERT_FALSE(guideline_aggregator->GetGuideline().ok());
  auto timestamp_ordered_noisy_keypoints = GetEllipticalGuideline();
  for (size_t t = 0; t < timestamp_ordered_noisy_keypoints.size(); ++t) {
    guideline_aggregator->AggregateKeypoints(
        timestamp_ordered_noisy_keypoints.at(t), t, Axis3::kZ);
  }
  GL_ASSERT_OK_AND_ASSIGN(auto guideline, guideline_aggregator->GetGuideline());
}

TEST(LocalTemporalRegressionBasedGuidelineAggregator, TestClearEntries) {
  GuidelineAggregatorOptions main_options;
  auto* options =
      main_options
          .mutable_local_temporal_regression_based_guideline_aggregator_options();  // NOLINT(whitespace/line_length)
  *options = GetDefaultLocalTemporalRegressionBasedGuidelineAggregatorOptions();
  options->set_time_interval_size(10);
  options->set_time_interval_overlap(2);
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator,
      LocalTemporalRegressionBasedGuidelineAggregator::Create(main_options));
  std::vector<std::vector<HitResult>> timestamp_ordered_noisy_keypoints =
      GetEllipticalGuideline();
  for (size_t t = 0; t < timestamp_ordered_noisy_keypoints.size(); ++t) {
    ASSERT_FALSE(guideline_aggregator->GetGuideline().ok());
    guideline_aggregator->AggregateKeypoints(
        timestamp_ordered_noisy_keypoints.at(t), t, Axis3::kZ);
    auto status_or_guideline = guideline_aggregator->GetGuideline();
    GL_EXPECT_OK(status_or_guideline.status());
    guideline_aggregator->ClearEntries(1).IgnoreError();
  }
  ASSERT_FALSE(guideline_aggregator->GetGuideline().ok());
}

TEST(LocalTemporalRegressionBasedGuidelineAggregator, TestInvalidVerticalAxes) {
  GuidelineAggregatorOptions main_options;
  auto* options =
      main_options
          .mutable_local_temporal_regression_based_guideline_aggregator_options();  // NOLINT(whitespace/line_length)
  *options = GetDefaultLocalTemporalRegressionBasedGuidelineAggregatorOptions();
  options->set_time_interval_size(10);
  options->set_time_interval_overlap(2);
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator,
      LocalTemporalRegressionBasedGuidelineAggregator::Create(main_options));
  ASSERT_FALSE(guideline_aggregator->GetGuideline().ok());
  auto timestamp_ordered_noisy_keypoints = GetStraightNoisyGuideline();
  guideline_aggregator->AggregateKeypoints(
      timestamp_ordered_noisy_keypoints.at(0), 0, Axis3::kY);
  GL_ASSERT_OK_AND_ASSIGN(auto guideline_points,
                          guideline_aggregator->GetGuideline());
  ASSERT_NEAR(guideline_points.front().z(), 0, 0.001);
  ASSERT_NEAR(guideline_points.back().z(), 0, 0.001);
}

TEST(LocalTemporalRegressionBasedGuidelineAggregator, TestGetGuideline1) {
  GuidelineAggregatorOptions main_options;
  auto* options =
      main_options
          .mutable_local_temporal_regression_based_guideline_aggregator_options();  // NOLINT(whitespace/line_length)
  *options = GetDefaultLocalTemporalRegressionBasedGuidelineAggregatorOptions();
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator,
      LocalTemporalRegressionBasedGuidelineAggregator::Create(main_options));
  ASSERT_FALSE(guideline_aggregator->GetGuideline().ok());
  std::vector<HitResult> hit_results;
  for (float i = 0; i <= 10; i++) {
    hit_results.push_back(HitResult(Transformation(Vector3d(i, 0, 0)), 0, 0.9));
  }
  guideline_aggregator->AggregateKeypoints(hit_results, 0, Axis3::kZ);
  GL_ASSERT_OK_AND_ASSIGN(auto guideline_points0,
                          guideline_aggregator->GetGuideline());
  ASSERT_EQ(guideline_points0.size(), 20);
  EXPECT_EIGEN_APPROX(guideline_points0.front(), Vector3d(-5, 0, 0));
  EXPECT_EIGEN_APPROX(guideline_points0.back(), Vector3d(14, 0, 0));

  options->set_num_guideline_points_per_meter(2);
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator1,
      LocalTemporalRegressionBasedGuidelineAggregator::Create(main_options));
  hit_results.clear();
  for (float i = 0; i <= 10; i++) {
    hit_results.push_back(HitResult(Transformation(Vector3d(i, 0, 0)), 0, 0.9));
  }
  guideline_aggregator1->AggregateKeypoints(hit_results, 1, Axis3::kZ);
  GL_ASSERT_OK_AND_ASSIGN(auto guideline_points1,
                          guideline_aggregator1->GetGuideline());
  ASSERT_EQ(guideline_points1.size(), 40);
  EXPECT_EIGEN_APPROX(guideline_points1.front(), Vector3d(-5, 0, 0));
  EXPECT_EIGEN_APPROX(guideline_points1.back(), Vector3d(14.5, 0, 0));
}

TEST(LocalTemporalRegressionBasedGuidelineAggregator, TestGetGuideline2) {
  GuidelineAggregatorOptions main_options;
  auto* options =
      main_options
          .mutable_local_temporal_regression_based_guideline_aggregator_options();  // NOLINT(whitespace/line_length)
  *options = GetDefaultLocalTemporalRegressionBasedGuidelineAggregatorOptions();
  GL_ASSERT_OK_AND_ASSIGN(
      auto guideline_aggregator,
      LocalTemporalRegressionBasedGuidelineAggregator::Create(main_options));
  ASSERT_FALSE(guideline_aggregator->GetGuideline().ok());
  std::vector<HitResult> hit_results;
  for (float i = 0; i <= 10; i++) {
    hit_results.push_back(HitResult(Transformation(Vector3d(i, 0, 0)), 0, 0.9));
    // Add these additional points to the sides of the previous point. The
    // regression algorithm will effectively eliminate these since the mean is
    // the previous point.
    hit_results.push_back(
        HitResult(Transformation(Vector3d(i, -1, -1)), 0, 0.9));
    hit_results.push_back(HitResult(Transformation(Vector3d(i, 1, 1)), 0, 0.9));
  }
  guideline_aggregator->AggregateKeypoints(hit_results, 0, Axis3::kZ);
  GL_ASSERT_OK_AND_ASSIGN(auto guideline_points0,
                          guideline_aggregator->GetGuideline());
  ASSERT_EQ(guideline_points0.size(), 20);

  EXPECT_EIGEN_APPROX(guideline_points0[10], Vector3d(5, 0, 0));
  EXPECT_EIGEN_APPROX(guideline_points0.front(), Vector3d(-5, 0, 0));
  EXPECT_EIGEN_APPROX(guideline_points0.back(), Vector3d(14, 0, 0));
}

}  // namespace
}  // namespace guideline::environment
