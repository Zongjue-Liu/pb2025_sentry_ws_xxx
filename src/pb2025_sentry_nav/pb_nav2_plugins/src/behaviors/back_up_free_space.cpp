// Copyright 2024 Polaris Xia
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

#include "pb_nav2_plugins/behaviors/back_up_free_space.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

namespace pb_nav2_behaviors
{

void BackUpFreeSpace::onConfigure()
{
  // Initialize the inherited collision checker, simulation horizon, publishers,
  // action server, frames, and other standard DriveOnHeading state first.
  nav2_behaviors::DriveOnHeading<nav2_msgs::action::BackUp>::onConfigure();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  const std::string parameter_prefix = behavior_name_ + ".";
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_prefix + "service_name", rclcpp::ParameterValue(service_name_));
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_prefix + "max_radius", rclcpp::ParameterValue(max_radius_));
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_prefix + "angle_increment", rclcpp::ParameterValue(angle_increment_));
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_prefix + "cost_threshold", rclcpp::ParameterValue(cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_prefix + "costmap_service_timeout",
    rclcpp::ParameterValue(costmap_service_timeout_));
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_prefix + "visualize", rclcpp::ParameterValue(visualize_));

  node->get_parameter(parameter_prefix + "service_name", service_name_);
  node->get_parameter(parameter_prefix + "max_radius", max_radius_);
  node->get_parameter(parameter_prefix + "angle_increment", angle_increment_);
  node->get_parameter(parameter_prefix + "cost_threshold", cost_threshold_);
  node->get_parameter(parameter_prefix + "costmap_service_timeout", costmap_service_timeout_);
  node->get_parameter(parameter_prefix + "visualize", visualize_);

  if (!std::isfinite(max_radius_) || max_radius_ < 0.0) {
    max_radius_ = 1.0;
  }
  if (!std::isfinite(angle_increment_) || angle_increment_ <= 0.0) {
    angle_increment_ = M_PI / 32.0;
  }
  angle_increment_ = std::clamp(angle_increment_, M_PI / 180.0, M_PI);
  cost_threshold_ = std::clamp(cost_threshold_, 0, 255);
  if (!std::isfinite(costmap_service_timeout_) || costmap_service_timeout_ < 0.0) {
    costmap_service_timeout_ = 1.0;
  }

  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  if (visualize_) {
    marker_pub_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_free_space_markers", 1);
    marker_pub_->on_activate();
  }
}

void BackUpFreeSpace::onCleanup()
{
  costmap_client_.reset();
  marker_pub_.reset();
  nav2_behaviors::DriveOnHeading<nav2_msgs::action::BackUp>::onCleanup();
}

nav2_behaviors::Status BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  if (
    !std::isfinite(command->target.x) || !std::isfinite(command->speed) ||
    command->target.x == 0.0 || command->speed == 0.0) {
    RCLCPP_ERROR(logger_, "Backup goal contains a zero or non-finite distance/speed");
    return nav2_behaviors::Status::FAILED;
  }

  const auto service_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(costmap_service_timeout_));
  if (!costmap_client_->wait_for_service(service_timeout)) {
    RCLCPP_ERROR(
      logger_, "Costmap service '%s' was not available within %.2f seconds", service_name_.c_str(),
      costmap_service_timeout_);
    return nav2_behaviors::Status::FAILED;
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto result = costmap_client_->async_send_request(request);
  if (result.wait_for(service_timeout) != std::future_status::ready) {
    RCLCPP_ERROR(
      logger_, "Costmap service '%s' did not respond within %.2f seconds", service_name_.c_str(),
      costmap_service_timeout_);
    return nav2_behaviors::Status::FAILED;
  }

  nav2_msgs::msg::Costmap costmap;
  try {
    const auto response = result.get();
    if (!response) {
      RCLCPP_ERROR(logger_, "Costmap service returned an empty response");
      return nav2_behaviors::Status::FAILED;
    }
    costmap = response->map;
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger_, "Costmap service failed: %s", exception.what());
    return nav2_behaviors::Status::FAILED;
  }

  if (!isCostmapValid(costmap)) {
    RCLCPP_ERROR(logger_, "Received an invalid or empty costmap");
    return nav2_behaviors::Status::FAILED;
  }

  const std::string search_frame =
    costmap.header.frame_id.empty() ? global_frame_ : costmap.header.frame_id;
  geometry_msgs::msg::PoseStamped search_pose;
  if (!nav2_util::getCurrentPose(
        search_pose, *tf_, search_frame, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(
      logger_, "Robot pose is not available in costmap frame '%s'", search_frame.c_str());
    return nav2_behaviors::Status::FAILED;
  }

  if (search_frame == global_frame_) {
    initial_pose_ = search_pose;
  } else if (!nav2_util::getCurrentPose(
               initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(
      logger_, "Robot pose is not available in behavior frame '%s'", global_frame_.c_str());
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::Pose2D pose;
  pose.x = search_pose.pose.position.x;
  pose.y = search_pose.pose.position.y;
  pose.theta = tf2::getYaw(search_pose.pose.orientation);

  geometry_msgs::msg::Pose2D collision_pose;
  collision_pose.x = initial_pose_.pose.position.x;
  collision_pose.y = initial_pose_.pose.position.y;
  collision_pose.theta = tf2::getYaw(initial_pose_.pose.orientation);

  const double requested_distance = std::abs(command->target.x);
  const double search_distance = std::min(requested_distance, max_radius_);
  if (search_distance <= 0.0) {
    RCLCPP_ERROR(logger_, "Backup search distance must be positive");
    return nav2_behaviors::Status::FAILED;
  }

  const auto candidates = findDirectionCandidates(costmap, pose, search_distance);
  if (candidates.empty()) {
    RCLCPP_WARN(logger_, "No free direction was found in the search costmap");
    return nav2_behaviors::Status::FAILED;
  }

  const double requested_speed = std::abs(command->speed);
  bool direction_selected = false;
  DirectionCandidate selected{};
  for (const auto & candidate : candidates) {
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = requested_speed * std::cos(candidate.body_angle);
    velocity.linear.y = requested_speed * std::sin(candidate.body_angle);
    if (isOmniTrajectoryCollisionFree(search_distance, velocity, collision_pose)) {
      twist_x_ = velocity.linear.x;
      twist_y_ = velocity.linear.y;
      selected = candidate;
      direction_selected = true;
      break;
    }
  }

  if (!direction_selected) {
    RCLCPP_WARN(
      logger_,
      "Free sectors exist in the search costmap, but none passed the local footprint check");
    return nav2_behaviors::Status::FAILED;
  }

  command_x_ = requested_distance;
  command_time_allowance_ = command->time_allowance;
  end_time_ = clock_->now() + command_time_allowance_;

  RCLCPP_INFO(
    logger_, "Escaping %.2f m at %.2f m/s: body %.1f deg, map %.1f deg, free sector %.1f deg",
    command_x_, requested_speed, selected.body_angle * 180.0 / M_PI,
    selected.map_angle * 180.0 / M_PI, selected.sector_width * 180.0 / M_PI);

  if (visualize_) {
    visualize(
      pose, search_frame, search_distance, selected.sector_center_map_angle, selected.sector_width);
  }

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status BackUpFreeSpace::onCycleUpdate()
{
  const rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(logger_, "Exceeded time allowance before reaching the escape goal");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available");
    return nav2_behaviors::Status::FAILED;
  }

  const double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  const double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  const double distance = std::hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance;
  action_server_->publish_feedback(feedback_);

  if (distance >= std::abs(command_x_)) {
    stopRobot();
    RCLCPP_INFO(logger_, "Free-space escape completed");
    return nav2_behaviors::Status::SUCCEEDED;
  }

  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = twist_x_;
  velocity.linear.y = twist_y_;

  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);

  const double speed = std::hypot(twist_x_, twist_y_);
  const double remaining_distance = std::max(0.0, std::abs(command_x_) - distance);
  const double cycle_distance = speed / std::max(cycle_frequency_, 1.0);
  const double lookahead_distance =
    std::max(cycle_distance, speed * std::max(simulate_ahead_time_, 0.0));
  const double collision_check_distance = std::min(remaining_distance, lookahead_distance);

  if (!isOmniTrajectoryCollisionFree(collision_check_distance, velocity, pose)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision ahead on the selected holonomic trajectory");
    return nav2_behaviors::Status::FAILED;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity);
  vel_pub_->publish(std::move(cmd_vel));
  return nav2_behaviors::Status::RUNNING;
}

bool BackUpFreeSpace::isCostmapValid(const nav2_msgs::msg::Costmap & costmap) const
{
  const auto size_x = static_cast<std::size_t>(costmap.metadata.size_x);
  const auto size_y = static_cast<std::size_t>(costmap.metadata.size_y);
  return std::isfinite(costmap.metadata.resolution) && costmap.metadata.resolution > 0.0 &&
         std::isfinite(costmap.metadata.origin.position.x) &&
         std::isfinite(costmap.metadata.origin.position.y) && size_x > 0 && size_y > 0 &&
         size_x <= std::numeric_limits<std::size_t>::max() / size_y &&
         costmap.data.size() == size_x * size_y;
}

std::vector<BackUpFreeSpace::DirectionCandidate> BackUpFreeSpace::findDirectionCandidates(
  const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
  double max_search_radius) const
{
  std::vector<DirectionCandidate> candidates;
  if (!isCostmapValid(costmap) || max_search_radius <= 0.0) {
    return candidates;
  }

  const std::size_t sample_count =
    std::max<std::size_t>(8, static_cast<std::size_t>(std::ceil(2.0 * M_PI / angle_increment_)));
  const double sample_step = 2.0 * M_PI / static_cast<double>(sample_count);
  std::vector<double> body_angles(sample_count);
  std::vector<bool> safe(sample_count, true);

  const double resolution = costmap.metadata.resolution;
  const double origin_x = costmap.metadata.origin.position.x;
  const double origin_y = costmap.metadata.origin.position.y;
  const auto size_x = static_cast<std::size_t>(costmap.metadata.size_x);
  const auto size_y = static_cast<std::size_t>(costmap.metadata.size_y);

  for (std::size_t i = 0; i < sample_count; ++i) {
    const double body_angle = -M_PI + static_cast<double>(i) * sample_step;
    const double map_angle = pose.theta + body_angle;
    body_angles[i] = body_angle;

    const std::size_t ray_sample_count =
      static_cast<std::size_t>(std::ceil(max_search_radius / resolution));
    for (std::size_t ray_sample = 0; ray_sample <= ray_sample_count; ++ray_sample) {
      const double radius =
        std::min(max_search_radius, static_cast<double>(ray_sample) * resolution);
      const double x = pose.x + radius * std::cos(map_angle);
      const double y = pose.y + radius * std::sin(map_angle);
      const auto map_x = static_cast<long>(std::floor((x - origin_x) / resolution));
      const auto map_y = static_cast<long>(std::floor((y - origin_y) / resolution));

      if (
        map_x < 0 || map_y < 0 || static_cast<std::size_t>(map_x) >= size_x ||
        static_cast<std::size_t>(map_y) >= size_y) {
        safe[i] = false;
        break;
      }

      const std::size_t index =
        static_cast<std::size_t>(map_y) * size_x + static_cast<std::size_t>(map_x);
      if (static_cast<int>(costmap.data[index]) > cost_threshold_) {
        safe[i] = false;
        break;
      }
    }
  }

  const auto first_unsafe = std::find(safe.begin(), safe.end(), false);
  if (first_unsafe == safe.end()) {
    // If every search ray is free, try conventional backward motion first,
    // then retain all other rays as local-footprint-checked fallbacks.
    candidates.push_back({M_PI, pose.theta + M_PI, pose.theta + M_PI, 2.0 * M_PI, 0.0});
    for (const double body_angle : body_angles) {
      const double center_error =
        std::abs(std::atan2(std::sin(body_angle - M_PI), std::cos(body_angle - M_PI)));
      if (center_error > 1e-9) {
        candidates.push_back(
          {body_angle, pose.theta + body_angle, pose.theta + M_PI, 2.0 * M_PI, center_error});
      }
    }

    std::sort(
      candidates.begin(), candidates.end(),
      [](const DirectionCandidate & lhs, const DirectionCandidate & rhs) {
        return lhs.center_error < rhs.center_error;
      });
    return candidates;
  }

  const std::size_t unsafe_index =
    static_cast<std::size_t>(std::distance(safe.begin(), first_unsafe));
  std::size_t offset = 1;
  while (offset <= sample_count) {
    const std::size_t index = (unsafe_index + offset) % sample_count;
    if (!safe[index]) {
      ++offset;
      continue;
    }

    const std::size_t run_start = offset;
    while (offset <= sample_count && safe[(unsafe_index + offset) % sample_count]) {
      ++offset;
    }

    const std::size_t run_length = offset - run_start;
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    for (std::size_t run_offset = run_start; run_offset < offset; ++run_offset) {
      const double angle = body_angles[(unsafe_index + run_offset) % sample_count];
      sum_sin += std::sin(angle);
      sum_cos += std::cos(angle);
    }

    const double body_angle = std::atan2(sum_sin, sum_cos);
    const double sector_width = static_cast<double>(run_length) * sample_step;
    const double sector_center_map_angle = pose.theta + body_angle;
    candidates.push_back(
      {body_angle, pose.theta + body_angle, sector_center_map_angle, sector_width, 0.0});

    for (std::size_t run_offset = run_start; run_offset < offset; ++run_offset) {
      const double fallback_angle = body_angles[(unsafe_index + run_offset) % sample_count];
      const double center_error = std::abs(
        std::atan2(std::sin(fallback_angle - body_angle), std::cos(fallback_angle - body_angle)));
      if (center_error > 1e-9) {
        candidates.push_back(
          {fallback_angle, pose.theta + fallback_angle, sector_center_map_angle, sector_width,
           center_error});
      }
    }
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const DirectionCandidate & lhs, const DirectionCandidate & rhs) {
      if (std::abs(lhs.sector_width - rhs.sector_width) > 1e-9) {
        return lhs.sector_width > rhs.sector_width;
      }
      if (std::abs(lhs.center_error - rhs.center_error) > 1e-9) {
        return lhs.center_error < rhs.center_error;
      }
      const auto backward_error = [](double angle) {
        return std::abs(std::atan2(std::sin(angle - M_PI), std::cos(angle - M_PI)));
      };
      return backward_error(lhs.body_angle) < backward_error(rhs.body_angle);
    });

  return candidates;
}

bool BackUpFreeSpace::isOmniTrajectoryCollisionFree(
  double max_distance, const geometry_msgs::msg::Twist & command,
  const geometry_msgs::msg::Pose2D & pose)
{
  if (!collision_checker_) {
    RCLCPP_ERROR(logger_, "Collision checker is not configured");
    return false;
  }

  const double speed = std::hypot(command.linear.x, command.linear.y);
  if (!std::isfinite(speed) || speed <= 0.0 || !std::isfinite(max_distance) || max_distance < 0.0) {
    return false;
  }

  const double step_distance = std::max(0.01, speed / std::max(cycle_frequency_, 1.0));
  const double cos_yaw = std::cos(pose.theta);
  const double sin_yaw = std::sin(pose.theta);
  bool fetch_costmap_and_footprint = true;
  const auto is_pose_collision_free = [&](double traveled) {
    const double travel_time = traveled / speed;
    geometry_msgs::msg::Pose2D projected_pose;
    projected_pose.x =
      pose.x + travel_time * (command.linear.x * cos_yaw - command.linear.y * sin_yaw);
    projected_pose.y =
      pose.y + travel_time * (command.linear.x * sin_yaw + command.linear.y * cos_yaw);
    projected_pose.theta = pose.theta;

    const bool collision_free =
      collision_checker_->isCollisionFree(projected_pose, fetch_costmap_and_footprint);
    fetch_costmap_and_footprint = false;
    return collision_free;
  };

  for (double traveled = 0.0; traveled < max_distance; traveled += step_distance) {
    if (!is_pose_collision_free(traveled)) {
      return false;
    }
  }

  return is_pose_collision_free(max_distance);
}

void BackUpFreeSpace::visualize(
  const geometry_msgs::msg::Pose2D & pose, const std::string & frame_id, double radius,
  double map_angle, double sector_width)
{
  if (!marker_pub_ || !marker_pub_->is_activated()) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker sector_marker;
  sector_marker.header.frame_id = frame_id;
  sector_marker.header.stamp = clock_->now();
  sector_marker.ns = "direction";
  sector_marker.id = 0;
  sector_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  sector_marker.action = visualization_msgs::msg::Marker::ADD;
  sector_marker.scale.x = 1.0;
  sector_marker.scale.y = 1.0;
  sector_marker.scale.z = 1.0;
  sector_marker.color.r = 0.0f;
  sector_marker.color.g = 1.0f;
  sector_marker.color.b = 0.0f;
  sector_marker.color.a = 0.2f;

  const double first_safe_angle = map_angle - sector_width / 2.0;
  const double last_safe_angle = map_angle + sector_width / 2.0;
  const double angle_step = std::max(0.02, angle_increment_);
  for (double angle = first_safe_angle; angle < last_safe_angle; angle += angle_step) {
    const double next_angle = std::min(angle + angle_step, last_safe_angle);

    geometry_msgs::msg::Point origin;
    origin.x = pose.x;
    origin.y = pose.y;

    geometry_msgs::msg::Point p1;
    p1.x = pose.x + radius * std::cos(angle);
    p1.y = pose.y + radius * std::sin(angle);

    geometry_msgs::msg::Point p2;
    p2.x = pose.x + radius * std::cos(next_angle);
    p2.y = pose.y + radius * std::sin(next_angle);

    sector_marker.points.push_back(origin);
    sector_marker.points.push_back(p1);
    sector_marker.points.push_back(p2);
  }
  markers.markers.push_back(sector_marker);

  auto create_arrow = [&](double angle, int id, float red, float green, float blue) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = frame_id;
    arrow.header.stamp = clock_->now();
    arrow.ns = "direction";
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = red;
    arrow.color.g = green;
    arrow.color.b = blue;
    arrow.color.a = 1.0;

    geometry_msgs::msg::Point start;
    start.x = pose.x;
    start.y = pose.y;

    geometry_msgs::msg::Point end;
    end.x = start.x + radius * std::cos(angle);
    end.y = start.y + radius * std::sin(angle);

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    return arrow;
  };

  markers.markers.push_back(create_arrow(first_safe_angle, 1, 0.0f, 0.0f, 1.0f));
  markers.markers.push_back(create_arrow(last_safe_angle, 2, 0.0f, 0.0f, 1.0f));
  markers.markers.push_back(create_arrow(map_angle, 3, 0.0f, 1.0f, 0.0f));
  marker_pub_->publish(markers);
}

}  // namespace pb_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::BackUpFreeSpace, nav2_core::Behavior)
