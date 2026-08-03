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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_

#include <chrono>
#include <cstdint>
#include <string>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{
/**
 * @brief A BT::ConditionNode that checks for the presence of an enemy target
 * returns SUCCESS if an enemy is detected
 */
class IsDetectEnemyCondition : public BT::SimpleConditionNode
{
public:
  IsDetectEnemyCondition(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Tick function for game status ports
   */
  BT::NodeStatus checkEnemy();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsDetectEnemyCondition");
  rclcpp::Clock clock_{RCL_SYSTEM_TIME};
  uint64_t last_motion_epoch_{0};
  uint64_t last_detector_sequence_{0};
  bool has_motion_epoch_{false};
  bool has_detector_sequence_{false};
  bool has_last_detection_{false};
  bool has_continuous_detection_{false};
  bool last_message_has_enemy_{false};
  std::chrono::steady_clock::time_point last_detector_update_time_;
  std::chrono::steady_clock::time_point last_detection_time_;
  std::chrono::steady_clock::time_point continuous_detection_start_time_;
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_
