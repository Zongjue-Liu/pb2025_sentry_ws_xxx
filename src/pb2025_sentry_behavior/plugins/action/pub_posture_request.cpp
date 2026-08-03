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

#include "pb2025_sentry_behavior/plugins/action/pub_posture_request.hpp"

namespace pb2025_sentry_behavior
{

PublishPostureRequestAction::PublishPostureRequestAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubNode(name, config, params)
{
}

BT::PortsList PublishPostureRequestAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<unsigned>("posture", 3, "Requested sentry posture ID (1-6)"),
  });
}

bool PublishPostureRequestAction::setMessage(pb_rm_interfaces::msg::SentryPostureCommand & message)
{
  unsigned posture = 0;
  if (!getInput("posture", posture) || posture < 1 || posture > 6) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid sentry posture request: %u", posture);
    return false;
  }
  message.posture = static_cast<uint8_t>(posture);
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishPostureRequestAction, "PublishPostureRequest");
