# pb_nav2_plugins

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins/actions/workflows/build_and_test.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins/actions/workflows/build_and_test.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. Overview

`pb_nav2_plugins` 是一个用于扩展 `Navigation2`（Nav2）框架的插件库，当前提供提供了一些额外的行为逻辑和代价地图层以增强机器人在导航过程中灵活性。

## 2. Plugins

### 2.1 Behaviors

#### 2.1.1 BackUpFreeSpace

`BackUpFreeSpace` 接管 Nav2 标准 `BackUp` action，无需额外的行为树节点。它会：

1. 从搜索代价地图中提取机器人周围连续的安全扇区。
2. 优先选择较宽的安全扇区；先检查扇区中心，再向两侧尝试候选方向，全向空旷时优先保持传统的后退方向。
3. 把地图坐标系方向转换为机器人本体坐标系的 `linear.x`、`linear.y`。
4. 使用局部代价地图和完整机器人 footprint 对候选轨迹及执行中的每个周期做碰撞检查。
5. 当代价地图、位姿、目标或碰撞检查无效时停止并返回失败，不盲目发送速度。

**Parameters:**

- `backup.service_name`: 用于搜索方向的代价地图服务（default：`global_costmap/get_costmap`）。
- `backup.max_radius`: 最大搜索距离（default：1.0 m）。
- `backup.angle_increment`: 方向采样间隔（default：pi / 32 rad）。
- `backup.cost_threshold`: 大于此值的代价值视为不安全（default：252）。
- `backup.costmap_service_timeout`: 等待代价地图服务及响应的最长时间（default：1.0 s）。
- `backup.visualize`: 是否在 RViz 中显示最终选中的安全扇区（default：false）。

通用 Nav2 参数 `simulate_ahead_time` 控制执行过程中局部 footprint 的滚动前视距离。

**Example:**

```yaml
behavior_server:
  ros__parameters:
    use_sim_time: true
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "pb_nav2_behaviors/BackUpFreeSpace"
      service_name: "global_costmap/get_costmap"
      max_radius: 2.0
      angle_increment: 0.09817477042468103
      cost_threshold: 252
      costmap_service_timeout: 1.0
      visualize: false
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"
    assisted_teleop:
      plugin: "nav2_behaviors/AssistedTeleop"
    local_frame: odom
    global_frame: map
    robot_base_frame: gimbal_yaw_fake
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

### 2.2 Layers

#### 2.2.1 IntensityVoxelLayer

`IntensityVoxelLayer` 是一个用于处理点云数据中障碍物强度信息的代价地图层。它可以根据点云数据的强度值来标记障碍物，并将这些障碍物信息添加到代价地图中，本插件推荐配合 [terrain_analysis](https://github.com/SMBU-PolarBear-Robotics-Team/terrain_analysis) 功能包使用。

**Parameters:**

Common to [costmap-plugins/voxel.html](https://docs.nav2.org/configuration/packages/costmap-plugins/voxel.html)。

主要区别在于添加了 `min_obstacle_intensity` 和 `max_obstacle_intensity` 参数，注意此参数使用域在 `obstacle_layer` 下，而非 `observation_sources` 下。

**Example:**

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: gimbal_yaw_fake
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      robot_radius: 0.2
      plugins: ["static_layer", "intensity_voxel_layer", "inflation_layer"]
      intensity_voxel_layer:
        plugin: pb_nav2_costmap_2d::IntensityVoxelLayer
        enabled: true
        track_unknown_space: true
        footprint_clearing_enabled: true
        publish_voxel_map: false
        combination_method: 1
        mark_threshold: 0
        origin_z: 0.0
        unknown_threshold: 5
        z_resolution: 0.05
        z_voxels: 16
        min_obstacle_height: 0.0
        max_obstacle_height: 2.0
        min_obstacle_intensity: 0.1
        max_obstacle_intensity: 2.0
        observation_sources: terrain_map
        terrain_map:
          data_type: PointCloud2
          topic: /terrain_map
          min_obstacle_height: 0.0
          max_obstacle_height: 2.0
          obstacle_max_range: 5.0
          obstacle_min_range: 0.2
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.0
        inflation_radius: 0.7
      always_send_full_costmap: False
```

## Acknowledgements

The Initial Developer of some parts of the repository (`BackUpFreeSpace`, `IntensityVoxelLayer`, `IntensityVoxelLayer`), which are copied from, derived from, or
inspired by @PolarisXQ [SCURM_SentryNavigation](https://github.com/PolarisXQ/SCURM_SentryNavigation/tree/master/nav2_plugins/behavior_ext_plugins), @ros-navigation [navigation2](https://github.com/ros-navigation).
All Rights Reserved.
