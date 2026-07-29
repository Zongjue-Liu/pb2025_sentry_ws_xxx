// Copyright 2025 PolarBear Robotics Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "pb_nav2_plugins/behaviors/back_up_free_space.hpp"

namespace pb_nav2_behaviors
{

class TestableBackUpFreeSpace : public BackUpFreeSpace
{
public:
  using BackUpFreeSpace::DirectionCandidate;
  using BackUpFreeSpace::findDirectionCandidates;

  void setSearchParameters(double angle_increment, int cost_threshold)
  {
    angle_increment_ = angle_increment;
    cost_threshold_ = cost_threshold;
  }
};

nav2_msgs::msg::Costmap makeCostmap(std::uint8_t initial_cost)
{
  nav2_msgs::msg::Costmap costmap;
  costmap.header.frame_id = "map";
  costmap.metadata.resolution = 0.1;
  costmap.metadata.size_x = 101;
  costmap.metadata.size_y = 101;
  costmap.metadata.origin.position.x = -5.05;
  costmap.metadata.origin.position.y = -5.05;
  costmap.data.assign(
    static_cast<std::size_t>(costmap.metadata.size_x) * costmap.metadata.size_y, initial_cost);
  return costmap;
}

void clearRay(nav2_msgs::msg::Costmap & costmap, double angle, double radius)
{
  const double resolution = costmap.metadata.resolution;
  const double step = resolution / 2.0;
  const std::size_t sample_count = static_cast<std::size_t>(std::ceil(radius / step));
  for (std::size_t sample = 0; sample <= sample_count; ++sample) {
    const double distance = std::min(radius, static_cast<double>(sample) * step);
    const double x = distance * std::cos(angle);
    const double y = distance * std::sin(angle);
    const auto map_x =
      static_cast<long>(std::floor((x - costmap.metadata.origin.position.x) / resolution));
    const auto map_y =
      static_cast<long>(std::floor((y - costmap.metadata.origin.position.y) / resolution));
    if (
      map_x >= 0 && map_y >= 0 && map_x < static_cast<long>(costmap.metadata.size_x) &&
      map_y < static_cast<long>(costmap.metadata.size_y)) {
      const auto index =
        static_cast<std::size_t>(map_y) * costmap.metadata.size_x + static_cast<std::size_t>(map_x);
      costmap.data[index] = 0;
    }
  }
}

TEST(BackUpFreeSpaceDirectionTest, ReturnsNoCandidateWhenEveryDirectionIsOccupied)
{
  TestableBackUpFreeSpace behavior;
  behavior.setSearchParameters(M_PI / 32.0, 252);
  const auto costmap = makeCostmap(254);
  geometry_msgs::msg::Pose2D pose;

  const auto candidates = behavior.findDirectionCandidates(costmap, pose, 1.0);

  EXPECT_TRUE(candidates.empty());
}

TEST(BackUpFreeSpaceDirectionTest, PrefersBackwardWhenEveryDirectionIsFree)
{
  TestableBackUpFreeSpace behavior;
  behavior.setSearchParameters(M_PI / 32.0, 252);
  const auto costmap = makeCostmap(0);
  geometry_msgs::msg::Pose2D pose;
  pose.theta = 0.4;

  const auto candidates = behavior.findDirectionCandidates(costmap, pose, 1.0);

  ASSERT_FALSE(candidates.empty());
  EXPECT_GT(candidates.size(), 1u);
  EXPECT_NEAR(std::abs(candidates.front().body_angle), M_PI, 1e-9);
  EXPECT_NEAR(candidates.front().map_angle, pose.theta + M_PI, 1e-9);
  EXPECT_NEAR(candidates.front().sector_width, 2.0 * M_PI, 1e-9);
}

TEST(BackUpFreeSpaceDirectionTest, ConvertsMapDirectionToRobotBodyDirection)
{
  TestableBackUpFreeSpace behavior;
  behavior.setSearchParameters(M_PI / 32.0, 252);
  auto costmap = makeCostmap(254);
  clearRay(costmap, 0.0, 1.0);
  geometry_msgs::msg::Pose2D pose;
  pose.theta = M_PI / 2.0;

  const auto candidates = behavior.findDirectionCandidates(costmap, pose, 1.0);

  ASSERT_FALSE(candidates.empty());
  EXPECT_NEAR(candidates.front().body_angle, -M_PI / 2.0, M_PI / 32.0);
  EXPECT_NEAR(candidates.front().map_angle, 0.0, M_PI / 32.0);
}

TEST(BackUpFreeSpaceDirectionTest, MergesSafeSectorAcrossMinusPiAndPi)
{
  TestableBackUpFreeSpace behavior;
  behavior.setSearchParameters(M_PI / 8.0, 252);
  auto costmap = makeCostmap(254);
  clearRay(costmap, -M_PI, 1.0);
  clearRay(costmap, -M_PI + M_PI / 8.0, 1.0);
  clearRay(costmap, M_PI - M_PI / 8.0, 1.0);
  geometry_msgs::msg::Pose2D pose;

  const auto candidates = behavior.findDirectionCandidates(costmap, pose, 1.0);

  ASSERT_FALSE(candidates.empty());
  EXPECT_NEAR(std::abs(candidates.front().body_angle), M_PI, M_PI / 8.0);
  EXPECT_GE(candidates.front().sector_width, 3.0 * M_PI / 8.0);
}

}  // namespace pb_nav2_behaviors
