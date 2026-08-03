// Copyright 2025 Lihan Chen
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

#include "pb2025_sentry_behavior/plugins/condition/is_emergency_stop.hpp"

namespace pb2025_sentry_behavior
{
IsEmergencyStopCondition::IsEmergencyStopCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsEmergencyStopCondition::checkEmergencyStop, this), config)
{
}

BT::PortsList IsEmergencyStopCondition::providedPorts()
{
  return {BT::InputPort<bool>("emergency_stop", "{@referee_emergencyStop}")};
}

BT::NodeStatus IsEmergencyStopCondition::checkEmergencyStop()
{
  const auto emergency_stop = getInput<bool>("emergency_stop");
  if (!emergency_stop) {
    return BT::NodeStatus::FAILURE;
  }
  return emergency_stop.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsEmergencyStopCondition>("IsEmergencyStop");
}
