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

#include "pb2025_sentry_behavior/goal_pose_nav_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace pb2025_sentry_behavior
{

GoalPoseNavBridge::GoalPoseNavBridge(const rclcpp::NodeOptions & options)
: Node("goal_pose_nav_bridge", options)
{
  goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
  game_status_topic_ = declare_parameter<std::string>("game_status_topic", "/referee/game_status");
  navigate_to_pose_action_ = declare_parameter<std::string>(
    "navigate_to_pose_action", "/red_standard_robot1/navigate_to_pose");
  reached_goal_topic_ =
    declare_parameter<std::string>("reached_goal_topic", "/navigation/reached_goal");
  position_tolerance_ = declare_parameter<double>("position_tolerance", 0.001);
  orientation_tolerance_ = declare_parameter<double>("orientation_tolerance", 0.001);
  const auto retry_period = declare_parameter<double>("retry_period", 0.2);
  const auto minimum_goal_update_period =
    declare_parameter<double>("minimum_goal_update_period", 0.5);

  if (
    position_tolerance_ < 0.0 || orientation_tolerance_ < 0.0 || retry_period <= 0.0 ||
    minimum_goal_update_period < 0.0) {
    throw std::invalid_argument(
      "Bridge tolerances and minimum_goal_update_period must be non-negative, and retry_period "
      "must be positive");
  }

  retry_period_ = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(retry_period));
  minimum_goal_update_period_ = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(minimum_goal_update_period));
  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_to_pose_action_);
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&GoalPoseNavBridge::goalPoseCallback, this, std::placeholders::_1));
  game_status_sub_ = create_subscription<pb_rm_interfaces::msg::GameStatus>(
    game_status_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&GoalPoseNavBridge::gameStatusCallback, this, std::placeholders::_1));
  reached_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    reached_goal_topic_, rclcpp::QoS(1).transient_local().reliable());
  retry_timer_ =
    create_wall_timer(retry_period_, std::bind(&GoalPoseNavBridge::processDesiredGoal, this));

  RCLCPP_INFO(
    get_logger(), "Bridging %s to action %s", goal_topic_.c_str(),
    navigate_to_pose_action_.c_str());
}

bool GoalPoseNavBridge::posesEquivalent(
  const geometry_msgs::msg::PoseStamped & lhs, const geometry_msgs::msg::PoseStamped & rhs,
  const double position_tolerance, const double orientation_tolerance)
{
  if (lhs.header.frame_id != rhs.header.frame_id) {
    return false;
  }

  const auto & lhs_position = lhs.pose.position;
  const auto & rhs_position = rhs.pose.position;
  const double dx = lhs_position.x - rhs_position.x;
  const double dy = lhs_position.y - rhs_position.y;
  const double dz = lhs_position.z - rhs_position.z;
  if (std::sqrt(dx * dx + dy * dy + dz * dz) > position_tolerance) {
    return false;
  }

  const auto & lhs_orientation = lhs.pose.orientation;
  const auto & rhs_orientation = rhs.pose.orientation;
  const double lhs_norm = std::sqrt(
    lhs_orientation.x * lhs_orientation.x + lhs_orientation.y * lhs_orientation.y +
    lhs_orientation.z * lhs_orientation.z + lhs_orientation.w * lhs_orientation.w);
  const double rhs_norm = std::sqrt(
    rhs_orientation.x * rhs_orientation.x + rhs_orientation.y * rhs_orientation.y +
    rhs_orientation.z * rhs_orientation.z + rhs_orientation.w * rhs_orientation.w);
  if (lhs_norm == 0.0 || rhs_norm == 0.0) {
    return false;
  }

  const double dot =
    (lhs_orientation.x * rhs_orientation.x + lhs_orientation.y * rhs_orientation.y +
     lhs_orientation.z * rhs_orientation.z + lhs_orientation.w * rhs_orientation.w) /
    (lhs_norm * rhs_norm);
  return 1.0 - std::abs(std::clamp(dot, -1.0, 1.0)) <= orientation_tolerance;
}

void GoalPoseNavBridge::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  auto normalized_goal = *msg;
  if (normalized_goal.header.frame_id.empty()) {
    normalized_goal.header.frame_id = "map";
  }
  normalized_goal.header.stamp = now();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_game_status_ || !game_running_) {
      return;
    }
    if (
      desired_goal_ &&
      posesEquivalent(
        normalized_goal, desired_goal_.value(), position_tolerance_, orientation_tolerance_)) {
      return;
    }
    desired_goal_ = normalized_goal;
    ++desired_revision_;
  }

  processDesiredGoal();
}

void GoalPoseNavBridge::gameStatusCallback(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg)
{
  bool state_changed = false;
  bool is_running = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool was_running = game_running_;
    has_game_status_ = true;
    game_running_ = msg->game_progress == pb_rm_interfaces::msg::GameStatus::RUNNING;
    is_running = game_running_;
    state_changed = was_running != game_running_;

    if (!game_running_ || state_changed) {
      desired_goal_.reset();
      completed_revision_ = 0;
      ++desired_revision_;
    }
    if (state_changed) {
      last_goal_dispatch_time_.reset();
    }
  }

  if (state_changed) {
    RCLCPP_INFO(
      get_logger(), "Navigation bridge is now %s for game progress %u",
      is_running ? "enabled" : "disabled", msg->game_progress);
    reached_goal_pub_->publish(geometry_msgs::msg::PoseStamped());
  }
  processDesiredGoal();
}

void GoalPoseNavBridge::processDesiredGoal()
{
  tryCancelOutdatedGoal();
  trySendDesiredGoal();
}

void GoalPoseNavBridge::tryCancelOutdatedGoal()
{
  GoalHandleNavigateToPose::SharedPtr goal_to_cancel;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!active_goal_handle_ || active_revision_ == desired_revision_ || cancel_in_progress_) {
      return;
    }
    const bool should_throttle = game_running_ && desired_goal_.has_value();
    if (
      should_throttle && last_goal_dispatch_time_ &&
      std::chrono::steady_clock::now() - last_goal_dispatch_time_.value() <
        minimum_goal_update_period_) {
      return;
    }
    goal_to_cancel = active_goal_handle_;
    cancel_in_progress_ = true;
  }

  action_client_->async_cancel_goal(
    goal_to_cancel,
    [this](const rclcpp_action::Client<NavigateToPose>::CancelResponse::SharedPtr response) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (response->goals_canceling.empty()) {
        cancel_in_progress_ = false;
        RCLCPP_WARN(get_logger(), "Navigation server rejected cancellation of the outdated goal");
      }
    });
}

void GoalPoseNavBridge::trySendDesiredGoal()
{
  NavigateToPose::Goal action_goal;
  uint64_t revision;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (
      !has_game_status_ || !game_running_ || !desired_goal_ || active_goal_handle_ ||
      send_in_progress_ || completed_revision_ == desired_revision_) {
      return;
    }
    const auto dispatch_time = std::chrono::steady_clock::now();
    if (
      last_goal_dispatch_time_ &&
      dispatch_time - last_goal_dispatch_time_.value() < minimum_goal_update_period_) {
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for navigation action server %s",
        navigate_to_pose_action_.c_str());
      return;
    }

    action_goal.pose = desired_goal_.value();
    action_goal.pose.header.stamp = now();
    revision = desired_revision_;
    send_in_progress_ = true;
    last_goal_dispatch_time_ = dispatch_time;
  }

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
    [this, revision](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
      handleGoalResponse(goal_handle, revision);
    };
  options.result_callback = [this,
                             revision](const GoalHandleNavigateToPose::WrappedResult & result) {
    handleResult(result, revision);
  };
  action_client_->async_send_goal(action_goal, options);
}

void GoalPoseNavBridge::handleGoalResponse(
  const GoalHandleNavigateToPose::SharedPtr & goal_handle, const uint64_t revision)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    send_in_progress_ = false;
    if (!goal_handle) {
      if (revision == desired_revision_) {
        desired_goal_.reset();
      }
      RCLCPP_WARN(get_logger(), "Navigation action server rejected the goal");
      return;
    }
    active_goal_handle_ = goal_handle;
    active_revision_ = revision;
  }

  RCLCPP_INFO(get_logger(), "Navigation goal revision %lu accepted", revision);
  processDesiredGoal();
}

void GoalPoseNavBridge::handleResult(
  const GoalHandleNavigateToPose::WrappedResult & result, const uint64_t revision)
{
  std::optional<geometry_msgs::msg::PoseStamped> reached_goal;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_goal_handle_.reset();
    active_revision_ = 0;
    cancel_in_progress_ = false;

    if (revision == desired_revision_) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        completed_revision_ = revision;
        reached_goal = desired_goal_;
      } else {
        desired_goal_.reset();
      }
    }
  }

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(get_logger(), "Navigation goal revision %lu succeeded", revision);
    if (reached_goal) {
      reached_goal->header.stamp = now();
      reached_goal_pub_->publish(reached_goal.value());
    }
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_INFO(get_logger(), "Navigation goal revision %lu canceled", revision);
  } else {
    RCLCPP_WARN(get_logger(), "Navigation goal revision %lu failed", revision);
  }
  processDesiredGoal();
}

}  // namespace pb2025_sentry_behavior
