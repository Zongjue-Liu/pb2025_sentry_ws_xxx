# Copyright 2026 SMBU PolarBear Robotics Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetLaunchConfiguration
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    robot_namespace = LaunchConfiguration("robot_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    detector = LaunchConfiguration("detector")
    dependent_start_delay = LaunchConfiguration("dependent_start_delay")
    behavior_start_delay = LaunchConfiguration("behavior_start_delay")
    start_behavior = LaunchConfiguration("start_behavior")

    simulator_share = get_package_share_directory("rmu_gazebo_simulator")
    navigation_share = get_package_share_directory("pb2025_nav_bringup")
    vision_share = get_package_share_directory("pb2025_vision_bringup")
    behavior_share = get_package_share_directory("pb2025_sentry_behavior")

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_share, "launch", "bringup_sim.launch.py")
        ),
        launch_arguments={"spawn_delay": "15.0", "world": "rmuc_2025"}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_share, "launch", "rm_navigation_simulation_launch.py"
            )
        ),
        launch_arguments={
            "namespace": robot_namespace,
            "world": "rmuc_2025",
            "slam": "False",
            "use_sim_time": use_sim_time,
            "use_rviz": "True",
        }.items(),
    )

    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vision_share, "launch", "rm_vision_simulation_launch.py")
        ),
        launch_arguments={
            "namespace": robot_namespace,
            "use_sim_time": use_sim_time,
            "detector": detector,
            "params_file": os.path.join(
                vision_share, "params", "simulation", "vision_params.yaml"
            ),
            "use_rviz": "False",
            "use_projectile_motion": "False",
            "use_hik_camera": "False",
            "use_composition": "False",
        }.items(),
    )

    control_panel = Node(
        package="pb_control_panel",
        executable="pb_control_panel",
        name="pb_control_panel",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "match_duration": 420,
                "posture_simulation_enabled": True,
            }
        ],
    )

    behavior = GroupAction(
        condition=IfCondition(start_behavior),
        scoped=True,
        actions=[
            SetLaunchConfiguration("namespace", ""),
            SetLaunchConfiguration("params_root_key", ""),
            SetLaunchConfiguration(
                "params_file",
                os.path.join(behavior_share, "params", "sentry_behavior.yaml"),
            ),
            SetLaunchConfiguration("robot_namespace", robot_namespace),
            SetLaunchConfiguration("use_sim_time", use_sim_time),
            SetLaunchConfiguration("target_tree", "rmuc2026_posture_strategy"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        behavior_share,
                        "launch",
                        "pb2025_sentry_behavior_launch.py",
                    )
                )
            ),
        ],
    )

    start_simulation = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ign",
                    "service",
                    "-s",
                    "/world/default/control",
                    "--reqtype",
                    "ignition.msgs.WorldControl",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    "10000",
                    "--req",
                    "pause: false",
                ],
                output="screen",
            )
        ],
    )

    dependent_systems = TimerAction(
        period=dependent_start_delay,
        actions=[navigation, vision, control_panel],
    )
    behavior_system = TimerAction(period=behavior_start_delay, actions=[behavior])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="red_standard_robot1",
                description="Backward-compatible autonomous robot namespace",
            ),
            DeclareLaunchArgument(
                "robot_namespace",
                default_value=namespace,
                description="Robot namespace used by navigation, vision, and TF",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use the Gazebo simulation clock",
            ),
            DeclareLaunchArgument(
                "detector",
                default_value="opencv",
                description="Armor detector used for simulation vision",
            ),
            DeclareLaunchArgument(
                "dependent_start_delay",
                default_value="25.0",
                description="Delay before navigation, vision, and referee panel start",
            ),
            DeclareLaunchArgument(
                "behavior_start_delay",
                default_value="35.0",
                description="Delay before the RMUC behavior tree starts",
            ),
            DeclareLaunchArgument(
                "start_behavior",
                default_value="True",
                description="Start the RMUC posture behavior tree",
            ),
            simulator,
            start_simulation,
            dependent_systems,
            behavior_system,
        ]
    )
