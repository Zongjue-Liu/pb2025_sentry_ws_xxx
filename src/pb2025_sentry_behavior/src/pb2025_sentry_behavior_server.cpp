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

#include "pb2025_sentry_behavior/pb2025_sentry_behavior_server.hpp"

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pb2025_sentry_behavior/custom_types.hpp"
#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "pb_rm_interfaces/msg/sentry_posture_status.hpp"
#include "std_msgs/msg/bool.hpp"
namespace pb2025_sentry_behavior
{

template <typename T>
void SentryBehaviorServer::subscribe(
  const std::string & topic, const std::string & bb_key, const rclcpp::QoS & qos)
{
  auto sub = node()->create_subscription<T>(
    topic, qos,
    [this, bb_key](const typename T::SharedPtr msg) { globalBlackboard()->set(bb_key, *msg); });
  subscriptions_.push_back(sub);
}

void SentryBehaviorServer::gameStatusCallback(
  const pb_rm_interfaces::msg::GameStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(referee_state_mutex_);
  const bool was_enabled = has_game_status_ &&
                           last_game_progress_ == pb_rm_interfaces::msg::GameStatus::RUNNING &&
                           !emergency_stop_;
  has_game_status_ = true;
  last_game_progress_ = msg->game_progress;
  const bool is_enabled =
    last_game_progress_ == pb_rm_interfaces::msg::GameStatus::RUNNING && !emergency_stop_;
  if (!was_enabled && is_enabled) {
    ++motion_epoch_;
  }
  globalBlackboard()->set("referee_gameStatus", *msg);
  globalBlackboard()->set("motion_epoch", motion_epoch_);
}

void SentryBehaviorServer::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(referee_state_mutex_);
  const bool was_enabled = has_game_status_ &&
                           last_game_progress_ == pb_rm_interfaces::msg::GameStatus::RUNNING &&
                           !emergency_stop_;
  emergency_stop_ = msg->data;
  const bool is_enabled = has_game_status_ &&
                          last_game_progress_ == pb_rm_interfaces::msg::GameStatus::RUNNING &&
                          !emergency_stop_;
  if (!was_enabled && is_enabled) {
    ++motion_epoch_;
  }
  globalBlackboard()->set("referee_emergencyStop", emergency_stop_);
  globalBlackboard()->set("motion_epoch", motion_epoch_);
}

void SentryBehaviorServer::detectorCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(referee_state_mutex_);
  ++detector_sequence_;
  globalBlackboard()->set("detector_armors", *msg);
  globalBlackboard()->set("detector_sequence", detector_sequence_);
}

SentryBehaviorServer::SentryBehaviorServer(const rclcpp::NodeOptions & options)
: TreeExecutionServer(options)
{
  node()->declare_parameter("use_cout_logger", false);
  node()->get_parameter("use_cout_logger", use_cout_logger_);
  const auto global_costmap_topic = node()->declare_parameter<std::string>(
    "global_costmap_topic", "/red_standard_robot1/global_costmap/costmap");
  const auto detector_topic = node()->declare_parameter<std::string>(
    "detector_topic", "/red_standard_robot1/detector/armors");
  const auto tracker_topic =
    node()->declare_parameter<std::string>("tracker_topic", "/red_standard_robot1/tracker/target");
  const auto posture_status_topic = node()->declare_parameter<std::string>(
    "posture_status_topic", "/referee/sentry_posture_status");
  const auto reached_goal_topic =
    node()->declare_parameter<std::string>("reached_goal_topic", "/navigation/reached_goal");

  subscribe<pb_rm_interfaces::msg::EventData>("referee/event_data", "referee_eventData");
  subscribe<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", "referee_allRobotHP");
  auto game_status_sub = node()->create_subscription<pb_rm_interfaces::msg::GameStatus>(
    "/referee/game_status", 10,
    std::bind(&SentryBehaviorServer::gameStatusCallback, this, std::placeholders::_1));
  subscriptions_.push_back(game_status_sub);
  subscribe<pb_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", "referee_groundRobotPosition");
  subscribe<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", "referee_rfidStatus");
  subscribe<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", "referee_robotStatus");
  subscribe<pb_rm_interfaces::msg::Buff>("referee/buff", "referee_buff");
  subscribe<pb_rm_interfaces::msg::SentryPostureStatus>(
    posture_status_topic, "referee_sentryPostureStatus");
  auto reached_goal_qos = rclcpp::QoS(1).transient_local().reliable();
  subscribe<geometry_msgs::msg::PoseStamped>(
    reached_goal_topic, "navigation_reachedGoal", reached_goal_qos);

  auto emergency_stop_sub = node()->create_subscription<std_msgs::msg::Bool>(
    "/referee/emergency_stop", 10,
    std::bind(&SentryBehaviorServer::emergencyStopCallback, this, std::placeholders::_1));
  subscriptions_.push_back(emergency_stop_sub);

  auto detector_qos = rclcpp::SensorDataQoS();
  auto detector_sub = node()->create_subscription<auto_aim_interfaces::msg::Armors>(
    detector_topic, detector_qos,
    std::bind(&SentryBehaviorServer::detectorCallback, this, std::placeholders::_1));
  subscriptions_.push_back(detector_sub);
  auto tracker_qos = rclcpp::SensorDataQoS();
  subscribe<auto_aim_interfaces::msg::Target>(tracker_topic, "tracker_target", tracker_qos);

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  subscribe<nav_msgs::msg::OccupancyGrid>(global_costmap_topic, "nav_globalCostmap", costmap_qos);
}

bool SentryBehaviorServer::onGoalReceived(
  const std::string & tree_name, const std::string & payload)
{
  RCLCPP_INFO(
    node()->get_logger(), "onGoalReceived with tree name '%s' with payload '%s'", tree_name.c_str(),
    payload.c_str());
  return true;
}

void SentryBehaviorServer::onTreeCreated(BT::Tree & tree)
{
  if (use_cout_logger_) {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }
  tick_count_ = 0;
}

std::optional<BT::NodeStatus> SentryBehaviorServer::onLoopAfterTick(BT::NodeStatus /*status*/)
{
  ++tick_count_;
  return std::nullopt;
}

std::optional<std::string> SentryBehaviorServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled)
{
  RCLCPP_INFO(
    node()->get_logger(), "onTreeExecutionCompleted with status=%d (canceled=%d) after %d ticks",
    static_cast<int>(status), was_cancelled, tick_count_);
  logger_cout_.reset();
  std::string result = treeName() +
                       " tree completed with status=" + std::to_string(static_cast<int>(status)) +
                       " after " + std::to_string(tick_count_) + " ticks";
  return result;
}

}  // namespace pb2025_sentry_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<pb2025_sentry_behavior::SentryBehaviorServer>(options);

  RCLCPP_INFO(action_server->node()->get_logger(), "Starting SentryBehaviorServer");

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();
}
