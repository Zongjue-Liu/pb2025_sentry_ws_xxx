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

#ifndef PB2025_SENTRY_BEHAVIOR__GOAL_POSE_NAV_BRIDGE_HPP_
#define PB2025_SENTRY_BEHAVIOR__GOAL_POSE_NAV_BRIDGE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace pb2025_sentry_behavior
{

class GoalPoseNavBridge : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit GoalPoseNavBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  static bool posesEquivalent(
    const geometry_msgs::msg::PoseStamped & lhs, const geometry_msgs::msg::PoseStamped & rhs,
    double position_tolerance, double orientation_tolerance);

private:
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void gameStatusCallback(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg);
  void processDesiredGoal();
  void tryCancelOutdatedGoal();
  void trySendDesiredGoal();
  void handleGoalResponse(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle, uint64_t revision);
  void handleResult(const GoalHandleNavigateToPose::WrappedResult & result, uint64_t revision);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reached_goal_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr retry_timer_;

  std::mutex state_mutex_;
  std::optional<geometry_msgs::msg::PoseStamped> desired_goal_;
  GoalHandleNavigateToPose::SharedPtr active_goal_handle_;
  uint64_t desired_revision_{0};
  uint64_t active_revision_{0};
  uint64_t completed_revision_{0};
  bool send_in_progress_{false};
  bool cancel_in_progress_{false};
  bool has_game_status_{false};
  bool game_running_{false};

  std::string goal_topic_;
  std::string game_status_topic_;
  std::string navigate_to_pose_action_;
  std::string reached_goal_topic_;
  double position_tolerance_;
  double orientation_tolerance_;
  std::chrono::milliseconds retry_period_;
  std::chrono::milliseconds minimum_goal_update_period_;
  std::optional<std::chrono::steady_clock::time_point> last_goal_dispatch_time_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__GOAL_POSE_NAV_BRIDGE_HPP_
