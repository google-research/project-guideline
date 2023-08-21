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

#include "project_guideline/environment/occupancy_map.h"

#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "Eigen/Core"
#include "project_guideline/depth/point_cloud_util.h"
#include "project_guideline/util/transformation.h"

namespace guideline {
namespace environment {

namespace {

using depth::Point3D;
using ::Eigen::Vector2d;
using ::Eigen::Vector3d;
using ::Eigen::Vector4d;
using ::testing::IsEmpty;
using ::testing::UnorderedElementsAreArray;
using util::Transformation;

std::vector<Vector2d> GetOccupiedGrids(
    std::vector<std::pair<Vector2d, int>> occupancy_map) {
  std::vector<Vector2d> occupied_grids;
  for (auto it = occupancy_map.begin(); it != occupancy_map.end(); ++it) {
    if (it->second > 0) {
      occupied_grids.push_back(it->first);
    }
  }
  return occupied_grids;
}

TEST(FrameBasedOccupancyMap, TestFilteredPoints) {
  OccupancyMapOptions options;
  options.frame_based_occupancy_map_options();
  options.mutable_frame_based_occupancy_map_options()
      ->mutable_clearance_zone_options()
      ->set_depth(2);
  options.mutable_frame_based_occupancy_map_options()
      ->mutable_clearance_zone_options()
      ->set_width(4);
  options.mutable_frame_based_occupancy_map_options()->set_occupancy_threshold(
      0);
  auto occupancy_map = OccupancyMap::Create(options);

  Transformation human_position_direction({0, 0, -0.707, -0.707},
                                          {0, 0, 1});
  std::vector<Point3D> point_cloud = {Point3D(Vector3d(-2.0, 1.2, -1.0), 0.6),
                                      Point3D(Vector3d(-2.0, 1.2, 3.0), 0.6),
                                      Point3D(Vector3d(-4.0, 1.0, 0.0), 0.9),
                                      Point3D(Vector3d(1.3, 0.2, 0.0), 0.05)};

  occupancy_map->UpdateOccupancyMap(point_cloud, human_position_direction);
  auto map = occupancy_map->GetOccupancyMap();
  EXPECT_EQ(map.size(), 20);
  EXPECT_THAT(GetOccupiedGrids(map), IsEmpty());
}

TEST(FrameBasedOccupancyMap, TestComputeOccupancyMap) {
  OccupancyMapOptions options;
  options.frame_based_occupancy_map_options();
  options.mutable_frame_based_occupancy_map_options()
      ->mutable_clearance_zone_options()
      ->set_depth(2);
  options.mutable_frame_based_occupancy_map_options()
      ->mutable_clearance_zone_options()
      ->set_width(4);
  options.mutable_frame_based_occupancy_map_options()->set_occupancy_threshold(
      2);
  auto occupancy_map = OccupancyMap::Create(options);

  Transformation human_position_direction({0, 0, -0.707, -0.707},
                                          {0, 0, 0});
  std::vector<Point3D> point_cloud = {Point3D(Vector3d(0.9, 0.8, 1.0), 0.5),
                                      Point3D(Vector3d(0.6, 0.6, 0.0), 0.8),
                                      Point3D(Vector3d(-2.0, 1.2, -0.4), 0.6),
                                      Point3D(Vector3d(-2.0, 1.0, 0.0), 0.9),
                                      Point3D(Vector3d(1.3, 0.2, 0.0), 0.5)};
  occupancy_map->UpdateOccupancyMap(point_cloud, human_position_direction);

  auto map = occupancy_map->GetOccupancyMap();
  EXPECT_EQ(map.size(), 20);
  EXPECT_THAT(GetOccupiedGrids(map),
              UnorderedElementsAreArray(
                  std::vector({Vector2d(1.0, 1.0), Vector2d(-2.0, 1.0)})));
}

TEST(FrameBasedOccupancyMap, TopBottomRelativeToCamera) {
  OccupancyMapOptions options;
  options.frame_based_occupancy_map_options();
  options.mutable_frame_based_occupancy_map_options()
      ->mutable_clearance_zone_options()
      ->set_depth(2);
  options.mutable_frame_based_occupancy_map_options()
      ->mutable_clearance_zone_options()
      ->set_width(4);
  // Top and Bottom are relative to camera position
  options.mutable_frame_based_occupancy_map_options()
      ->mutable_clearance_zone_options()
      ->set_bottom(-0.2);
  options.mutable_frame_based_occupancy_map_options()
      ->mutable_clearance_zone_options()
      ->set_top(0.5);
  options.mutable_frame_based_occupancy_map_options()->set_occupancy_threshold(
      1);
  auto occupancy_map = OccupancyMap::Create(options);

  Transformation human_position_direction({0, 0, -0.707, -0.707},
                                          {0, 0, 4});
  std::vector<Point3D> point_cloud = {Point3D(Vector3d(0.9, 0.8, 4.4), 1.0),
                                      Point3D(Vector3d(1.9, -0.8, 3.85), 1.0),
                                      Point3D(Vector3d(-2.0, 1.2, 4.6), 1.0),
                                      Point3D(Vector3d(-2.0, 1.0, 3.2), 1.0),
                                      Point3D(Vector3d(1.3, 0.2, 0.5), 1.0)};
  occupancy_map->UpdateOccupancyMap(point_cloud, human_position_direction);

  auto map = occupancy_map->GetOccupancyMap();
  EXPECT_EQ(map.size(), 20);
  EXPECT_THAT(GetOccupiedGrids(map),
              UnorderedElementsAreArray(
                  std::vector({Vector2d(1.0, 1.0), Vector2d(2.0, -1.0)})));
}

}  // namespace
}  // namespace environment
}  // namespace guideline
