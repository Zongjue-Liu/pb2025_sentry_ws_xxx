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

#include "pb2025_sentry_behavior/plugins/condition/is_posture_available.hpp"

#include "pb_rm_interfaces/msg/sentry_posture_status.hpp"

namespace pb2025_sentry_behavior
{

IsPostureAvailableCondition::IsPostureAvailableCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsPostureAvailableCondition::checkPosture, this), config)
{
}

BT::PortsList IsPostureAvailableCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::SentryPostureStatus>(
      "status", "{@referee_sentryPostureStatus}", "Confirmed referee posture status"),
    BT::InputPort<unsigned>("posture", 3, "Requested posture ID (1-6)"),
    BT::InputPort<double>("minimum_enhanced_time", 0.1, "Required enhanced allowance in seconds"),
  };
}

BT::NodeStatus IsPostureAvailableCondition::checkPosture()
{
  const auto status = getInput<pb_rm_interfaces::msg::SentryPostureStatus>("status");
  unsigned posture = 0;
  double minimum_enhanced_time = 0.1;
  if (
    !status || !getInput("posture", posture) ||
    !getInput("minimum_enhanced_time", minimum_enhanced_time)) {
    return BT::NodeStatus::FAILURE;
  }
  if (posture < 1 || posture > 6) {
    return BT::NodeStatus::FAILURE;
  }

  const unsigned effective_posture = status->posture_id + (status->is_powered ? 3U : 0U);
  if (effective_posture == posture) {
    return BT::NodeStatus::SUCCESS;
  }
  if (status->switch_cooldown_remaining > 0.0F) {
    return BT::NodeStatus::FAILURE;
  }
  if (posture <= 3) {
    return BT::NodeStatus::SUCCESS;
  }

  double remaining = 0.0;
  if (posture == pb_rm_interfaces::msg::SentryPostureStatus::ENHANCED_ATTACK) {
    remaining = status->enhanced_attack_remaining_time;
  } else if (posture == pb_rm_interfaces::msg::SentryPostureStatus::ENHANCED_DEFENSE) {
    remaining = status->enhanced_defense_remaining_time;
  } else {
    remaining = status->enhanced_mobile_remaining_time;
  }
  return remaining >= minimum_enhanced_time ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsPostureAvailableCondition>(
    "IsPostureAvailable");
}
