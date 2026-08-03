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

#include "pb2025_sentry_behavior/plugins/condition/is_detect_enemy.hpp"

namespace pb2025_sentry_behavior
{

IsDetectEnemyCondition::IsDetectEnemyCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsDetectEnemyCondition::checkEnemy, this), config)
{
}

BT::PortsList IsDetectEnemyCondition::providedPorts()
{
  return {
    BT::InputPort<auto_aim_interfaces::msg::Armors>(
      "key_port", "{@detector_armors}", "Vision detector port on blackboard"),
    BT::InputPort<std::vector<int>>(
      "armor_id", "1;2;3;4;5;7",
      "Expected id of armors. Multiple numbers should be separated by the character `;` in Groot2"),
    BT::InputPort<float>("max_distance", 8.0, "Distance to enemy target"),
    BT::InputPort<float>("lost_timeout", 0.0, "Keep detecting after target loss, in seconds"),
    BT::InputPort<float>("message_timeout", 0.5, "Maximum detector message age, in seconds"),
    BT::InputPort<float>(
      "minimum_detection_time", 0.0, "Required continuous detection time, in seconds"),
    BT::InputPort<float>(
      "detection_gap_tolerance", 0.0,
      "Maximum empty-frame gap that does not reset continuous detection, in seconds"),
    BT::InputPort<uint64_t>("detector_sequence", "{@detector_sequence}"),
    BT::InputPort<uint64_t>("motion_epoch", "{@motion_epoch}"),
  };
}

BT::NodeStatus IsDetectEnemyCondition::checkEnemy()
{
  std::vector<int> expected_armor_ids;
  float max_distance;
  float lost_timeout;
  float message_timeout;
  float minimum_detection_time;
  float detection_gap_tolerance;
  uint64_t detector_sequence;
  uint64_t motion_epoch;
  auto msg = getInput<auto_aim_interfaces::msg::Armors>("key_port");
  if (
    !msg || !getInput("detector_sequence", detector_sequence) ||
    !getInput("motion_epoch", motion_epoch)) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 2000,
      "Detector messages have not been received; treating the current state as no enemy");
    return BT::NodeStatus::FAILURE;
  }

  getInput("armor_id", expected_armor_ids);
  getInput("max_distance", max_distance);
  getInput("lost_timeout", lost_timeout);
  getInput("message_timeout", message_timeout);
  getInput("minimum_detection_time", minimum_detection_time);
  getInput("detection_gap_tolerance", detection_gap_tolerance);

  const auto now = std::chrono::steady_clock::now();
  if (!has_motion_epoch_) {
    has_motion_epoch_ = true;
    last_motion_epoch_ = motion_epoch;
  } else if (motion_epoch != last_motion_epoch_) {
    last_motion_epoch_ = motion_epoch;
    last_detector_sequence_ = detector_sequence;
    has_detector_sequence_ = true;
    has_last_detection_ = false;
    has_continuous_detection_ = false;
    last_message_has_enemy_ = false;
    return BT::NodeStatus::FAILURE;
  }

  if (!has_detector_sequence_) {
    has_detector_sequence_ = true;
    last_detector_sequence_ = detector_sequence > 0 ? detector_sequence - 1 : 0;
  }

  if (detector_sequence != last_detector_sequence_) {
    if (
      has_continuous_detection_ &&
      std::chrono::duration<double>(now - last_detector_update_time_).count() > message_timeout) {
      has_continuous_detection_ = false;
    }
    last_detector_sequence_ = detector_sequence;
    last_detector_update_time_ = now;
    last_message_has_enemy_ = false;

    for (const auto & armor : msg->armors) {
      const float distance_to_enemy =
        std::hypot(armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);

      if (armor.number.empty()) {
        continue;
      }
      int armor_id = std::stoi(armor.number);
      const bool is_armor_id_match =
        std::find(expected_armor_ids.begin(), expected_armor_ids.end(), armor_id) !=
        expected_armor_ids.end();

      const bool is_within_distance = (distance_to_enemy <= max_distance);

      if (is_armor_id_match && is_within_distance) {
        last_message_has_enemy_ = true;
        has_last_detection_ = true;
        last_detection_time_ = now;
        if (!has_continuous_detection_) {
          has_continuous_detection_ = true;
          continuous_detection_start_time_ = now;
        }
        break;
      }
    }
    if (
      !last_message_has_enemy_ &&
      (!has_last_detection_ || detection_gap_tolerance <= 0.0F ||
       std::chrono::duration<double>(now - last_detection_time_).count() >
         detection_gap_tolerance)) {
      has_continuous_detection_ = false;
    }
  }

  const bool detector_message_is_fresh =
    std::chrono::duration<double>(now - last_detector_update_time_).count() <= message_timeout;
  if (last_message_has_enemy_ && detector_message_is_fresh) {
    const bool stable_long_enough =
      minimum_detection_time <= 0.0F ||
      std::chrono::duration<double>(now - continuous_detection_start_time_).count() >=
        minimum_detection_time;
    return stable_long_enough ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  if (!detector_message_is_fresh) {
    has_continuous_detection_ = false;
  }

  if (
    has_continuous_detection_ && has_last_detection_ && detection_gap_tolerance > 0.0F &&
    std::chrono::duration<double>(now - last_detection_time_).count() > detection_gap_tolerance) {
    has_continuous_detection_ = false;
  }

  if (
    has_last_detection_ && lost_timeout > 0.0F &&
    std::chrono::duration<double>(now - last_detection_time_).count() <= lost_timeout) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsDetectEnemyCondition>("IsDetectEnemy");
}
