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

#ifndef PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
#define PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using BackUpAction = nav2_msgs::action::BackUp;

namespace pb_nav2_behaviors
{

/**
 * @class pb_nav2_behaviors::BackUpFreeSpace
 * @brief Selects a free holonomic escape direction and validates the full robot footprint.
 */
class BackUpFreeSpace : public nav2_behaviors::DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  BackUpFreeSpace() = default;

  void onConfigure() override;
  void onCleanup() override;
  nav2_behaviors::Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  struct DirectionCandidate
  {
    double body_angle;
    double map_angle;
    double sector_center_map_angle;
    double sector_width;
    double center_error;
  };

  std::vector<DirectionCandidate> findDirectionCandidates(
    const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
    double max_search_radius) const;

  bool isCostmapValid(const nav2_msgs::msg::Costmap & costmap) const;

  bool isOmniTrajectoryCollisionFree(
    double max_distance, const geometry_msgs::msg::Twist & command,
    const geometry_msgs::msg::Pose2D & pose);

  void visualize(
    const geometry_msgs::msg::Pose2D & pose, const std::string & frame_id, double radius,
    double map_angle, double sector_width);

  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
    marker_pub_;

  double twist_x_{0.0};
  double twist_y_{0.0};

  // Parameters under the "backup" behavior namespace.
  std::string service_name_{"global_costmap/get_costmap"};
  double max_radius_{1.0};
  double angle_increment_{0.09817477042468103};  // pi / 32
  int cost_threshold_{252};
  double costmap_service_timeout_{1.0};
  bool visualize_{false};
};

}  // namespace pb_nav2_behaviors

#endif  // PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
