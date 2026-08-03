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

#include "fake_vel_transform/fake_vel_transform.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

constexpr double EPSILON = 1e-5;
constexpr double CONTROLLER_TIMEOUT = 0.5;

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("cmd_spin_topic", "/cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<std::string>("game_status_topic", "/referee/game_status");
  this->declare_parameter<std::string>("emergency_stop_topic", "/referee/emergency_stop");
  this->declare_parameter<float>("init_spin_speed", 0.0);
  this->declare_parameter<double>("cmd_vel_timeout", 0.3);
  this->declare_parameter<double>("referee_timeout", 2.0);
  this->declare_parameter<double>("output_frequency", 50.0);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("game_status_topic", game_status_topic_);
  this->get_parameter("emergency_stop_topic", emergency_stop_topic_);
  this->get_parameter("init_spin_speed", spin_speed_);
  this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
  this->get_parameter("referee_timeout", referee_timeout_);
  this->get_parameter("output_frequency", output_frequency_);

  if (cmd_vel_timeout_ <= 0.0 || referee_timeout_ <= 0.0 || output_frequency_ <= 0.0) {
    throw std::invalid_argument("Timeouts and output_frequency must be greater than zero");
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  game_status_sub_ = this->create_subscription<pb_rm_interfaces::msg::GameStatus>(
    game_status_topic_, 10,
    std::bind(&FakeVelTransform::gameStatusCallback, this, std::placeholders::_1));
  emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    emergency_stop_topic_, 10,
    std::bind(&FakeVelTransform::emergencyStopCallback, this, std::placeholders::_1));

  odom_sub_filter_.subscribe(this, odom_topic_);
  local_plan_sub_filter_.subscribe(this, local_plan_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  local_plan_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::localPlanCallback, this, std::placeholders::_1));

  // In Navigation2 Humble release, the velocity is published by the controller without timestamped.
  // We consider the velocity is published at the same time as local_plan.
  // Therefore, we use ApproximateTime policy to synchronize `cmd_vel` and `odometry`.
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(
    std::bind(&FakeVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  // 50Hz Timer to send transform from `robot_base_frame` to `fake_robot_base_frame`
  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransform, this));
  const auto velocity_period = std::chrono::duration<double>(1.0 / output_frequency_);
  velocity_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(velocity_period),
    std::bind(&FakeVelTransform::publishVelocity, this));
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  spin_speed_ = msg->data;
}

void FakeVelTransform::gameStatusCallback(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  has_game_status_ = true;
  last_game_status_time_ = std::chrono::steady_clock::now();
  game_running_ = msg->game_progress == pb_rm_interfaces::msg::GameStatus::RUNNING;
  if (!game_running_) {
    latest_cmd_vel_.reset();
  }
}

void FakeVelTransform::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  emergency_stop_ = msg->data;
  if (emergency_stop_) {
    latest_cmd_vel_.reset();
  }
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // NOTE: Haven't synced with local_plan
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto controller_age =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - last_controller_activate_time_)
      .count();
  if (controller_age > CONTROLLER_TIMEOUT) {
    current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
  }
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const bool is_zero_vel = std::abs(msg->linear.x) < EPSILON && std::abs(msg->linear.y) < EPSILON &&
                           std::abs(msg->angular.z) < EPSILON;
  if (is_zero_vel) {
    latest_cmd_vel_.reset();
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const bool referee_is_fresh =
    has_game_status_ &&
    std::chrono::duration<double>(now - last_game_status_time_).count() <= referee_timeout_;
  if (!referee_is_fresh || !game_running_ || emergency_stop_) {
    latest_cmd_vel_.reset();
    return;
  }

  latest_cmd_vel_ = msg;
  last_cmd_vel_time_ = now;
}

void FakeVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  // Consider nav2_controller_server is activated when receiving local_plan
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_controller_activate_time_ = std::chrono::steady_clock::now();
}

void FakeVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Path::ConstSharedPtr & /*local_plan_msg*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_robot_base_angle_ = tf2::getYaw(odom_msg->pose.pose.orientation);
}

void FakeVelTransform::publishTransform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    q.setRPY(0, 0, -current_robot_base_angle_);
  }
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

void FakeVelTransform::publishVelocity()
{
  geometry_msgs::msg::Twist output;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = std::chrono::steady_clock::now();
    const bool referee_is_fresh =
      has_game_status_ &&
      std::chrono::duration<double>(now - last_game_status_time_).count() <= referee_timeout_;
    const bool motion_enabled = referee_is_fresh && game_running_ && !emergency_stop_;

    if (!motion_enabled) {
      latest_cmd_vel_.reset();
    } else {
      if (
        latest_cmd_vel_ &&
        std::chrono::duration<double>(now - last_cmd_vel_time_).count() <= cmd_vel_timeout_) {
        output = transformVelocity(*latest_cmd_vel_, current_robot_base_angle_);
      } else {
        latest_cmd_vel_.reset();
        output.angular.z = spin_speed_;
      }
    }
  }
  cmd_vel_chassis_pub_->publish(output);
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist & twist, float yaw_diff) const
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = twist.angular.z + spin_speed_;
  aft_tf_vel.linear.x = twist.linear.x * cos(yaw_diff) + twist.linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist.linear.x * sin(yaw_diff) + twist.linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
