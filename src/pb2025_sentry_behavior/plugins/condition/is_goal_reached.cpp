// Copyright 2026 SMBU PolarBear Robotics Team
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

#include "pb2025_sentry_behavior/plugins/condition/is_goal_reached.hpp"

#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

IsGoalReachedCondition::IsGoalReachedCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsGoalReachedCondition::checkGoal, this), config)
{
}

BT::PortsList IsGoalReachedCondition::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "reached_goal", "{@navigation_reachedGoal}", "Last navigation goal completed successfully"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "0;0;0", "Expected completed goal"),
    BT::InputPort<double>("position_tolerance", 0.2, "Goal position comparison tolerance"),
  };
}

BT::NodeStatus IsGoalReachedCondition::checkGoal()
{
  const auto reached = getInput<geometry_msgs::msg::PoseStamped>("reached_goal");
  const auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  double tolerance = 0.2;
  if (!reached || !goal || !getInput("position_tolerance", tolerance)) {
    return BT::NodeStatus::FAILURE;
  }
  if (reached->header.frame_id.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  const double dx = reached->pose.position.x - goal->pose.position.x;
  const double dy = reached->pose.position.y - goal->pose.position.y;
  return std::hypot(dx, dy) <= tolerance ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsGoalReachedCondition>("IsGoalReached");
}
