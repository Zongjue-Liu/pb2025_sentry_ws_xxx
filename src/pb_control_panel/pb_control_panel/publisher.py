from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
from pb_rm_interfaces.msg import Buff
from pb_rm_interfaces.msg import EventData
from pb_rm_interfaces.msg import GameRobotHP
from pb_rm_interfaces.msg import GameStatus
from pb_rm_interfaces.msg import GroundRobotPosition
from pb_rm_interfaces.msg import RfidStatus
from pb_rm_interfaces.msg import RobotStatus
from pb_rm_interfaces.msg import SentryPostureCommand
from pb_rm_interfaces.msg import SentryPostureStatus
from rmoss_interfaces.msg import RefereeCmd
from std_msgs.msg import Bool

from pb_control_panel.posture import SentryPostureStateMachine


class PbControlPanelPublisher(Node):
    def __init__(self, node_name="pb_control_panel", parameter_overrides=None):
        super().__init__(
            node_name,
            parameter_overrides=parameter_overrides or [],
        )

        self.game_status_pub = self.create_publisher(
            GameStatus, "referee/game_status", 10
        )
        self.robot_status_pub = self.create_publisher(
            RobotStatus, "referee/robot_status", 10
        )
        self.rfid_status_pub = self.create_publisher(
            RfidStatus, "referee/rfid_status", 10
        )
        self.game_robot_hp_pub = self.create_publisher(
            GameRobotHP, "referee/all_robot_hp", 10
        )
        self.event_data_pub = self.create_publisher(EventData, "referee/event_data", 10)
        self.buff_pub = self.create_publisher(Buff, "referee/buff", 10)
        self.ground_robot_position_pub = self.create_publisher(
            GroundRobotPosition, "referee/ground_robot_position", 10
        )
        self.emergency_stop_pub = self.create_publisher(
            Bool, "referee/emergency_stop", 10
        )
        self.referee_cmd_pub = self.create_publisher(
            RefereeCmd, "/referee_system/referee_cmd", 10
        )
        self.sentry_posture_status_pub = self.create_publisher(
            SentryPostureStatus, "/referee/sentry_posture_status", 10
        )
        self.sentry_posture_command_sub = self.create_subscription(
            SentryPostureCommand,
            "/referee/sentry_posture_command",
            self._posture_command_callback,
            10,
        )

        self.match_duration = self.declare_parameter("match_duration", 300).value
        self.posture_simulation_enabled = self.declare_parameter(
            "posture_simulation_enabled", False
        ).value
        if self.match_duration <= 0:
            raise ValueError("match_duration must be positive")

        initial_pose_topic = self.declare_parameter(
            "initial_pose_topic", "/red_standard_robot1/initialpose"
        ).value
        global_costmap_clear_service = self.declare_parameter(
            "global_costmap_clear_service",
            "/red_standard_robot1/global_costmap/clear_entirely_global_costmap",
        ).value
        local_costmap_clear_service = self.declare_parameter(
            "local_costmap_clear_service",
            "/red_standard_robot1/local_costmap/clear_entirely_local_costmap",
        ).value
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, initial_pose_topic, 10
        )
        self.costmap_clear_clients = [
            self.create_client(ClearEntireCostmap, global_costmap_clear_service),
            self.create_client(ClearEntireCostmap, local_costmap_clear_service),
        ]

        self.game_status = GameStatus()
        self.game_status.game_progress = GameStatus.NOT_START
        self.game_status.stage_remain_time = self.match_duration
        self.emergency_stop = Bool()
        self.emergency_stop.data = False

        self.robot_status = RobotStatus()
        self.robot_status.robot_id = 7
        self.robot_status.robot_level = 1
        self.robot_status.current_hp = 400
        self.robot_status.maximum_hp = 400
        self.robot_status.shooter_barrel_cooling_value = 30
        self.robot_status.shooter_barrel_heat_limit = 260
        self.robot_status.shooter_17mm_1_barrel_heat = 0
        self.robot_status.armor_id = 0
        self.robot_status.hp_deduction_reason = RobotStatus.ARMOR_HIT
        self.robot_status.projectile_allowance_17mm = 750
        self.robot_status.remaining_gold_coin = 0
        self.robot_status.is_hp_deduced = False

        self.rfid_status = RfidStatus()
        self.game_robot_hp = GameRobotHP()
        self.event_data = EventData()
        self.buff = Buff()
        self.ground_robot_position = GroundRobotPosition()
        self.posture_state = SentryPostureStateMachine()
        self.sentry_posture_status = SentryPostureStatus()
        self.posture_state.fill_status_message(self.sentry_posture_status)
        self.manual_posture_override = False
        self._manual_override_warning_emitted = False

        self.countdown_enabled = False

    def set_game_status(self, game_progress, stage_remain_time):
        self.game_status.game_progress = game_progress
        self.game_status.stage_remain_time = stage_remain_time

    def set_emergency_stop(self, enabled):
        self.emergency_stop.data = enabled

    def set_manual_posture_override(self, enabled):
        self.manual_posture_override = bool(enabled)
        self._manual_override_warning_emitted = False

    def start_preparation(self):
        self.posture_state.reset_match()
        self.game_status.game_progress = GameStatus.PREPARATION
        self.game_status.stage_remain_time = 0
        self.countdown_enabled = False
        self._publish_referee_command(RefereeCmd.START_PREPARATION)

    def start_self_checking(self):
        self.game_status.game_progress = GameStatus.SELF_CHECKING
        self.game_status.stage_remain_time = 15
        self.countdown_enabled = False
        self._publish_referee_command(RefereeCmd.START_SELF_CHECKING)
        self.reset_navigation()

    def start_countdown(self):
        self.game_status.game_progress = GameStatus.COUNT_DOWN
        self.game_status.stage_remain_time = 5
        self.countdown_enabled = False
        self.reset_navigation()

    def start_match(self):
        self.game_status.game_progress = GameStatus.RUNNING
        self.game_status.stage_remain_time = self.match_duration
        self.countdown_enabled = True
        self._publish_referee_command(RefereeCmd.START_GAME)

    def end_match(self):
        self.game_status.game_progress = GameStatus.GAME_OVER
        self.game_status.stage_remain_time = 0
        self.countdown_enabled = False
        self._publish_referee_command(RefereeCmd.STOP_GAME)

    def reset_navigation(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.orientation.w = 1.0
        initial_pose.pose.covariance[0] = 0.25
        initial_pose.pose.covariance[7] = 0.25
        initial_pose.pose.covariance[35] = 0.0685
        self.initial_pose_pub.publish(initial_pose)

        for client in self.costmap_clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
            else:
                self.get_logger().warning(
                    "Costmap clear service is not ready: {}".format(client.srv_name)
                )

    def _publish_referee_command(self, command):
        message = RefereeCmd()
        message.cmd = command
        self.referee_cmd_pub.publish(message)

    def set_robot_status(
        self,
        current_hp,
        maximum_hp,
        heat,
        heat_limit,
        projectile_allowance_17mm,
        is_hp_deduced,
        armor_id,
    ):
        self.robot_status.current_hp = current_hp
        self.robot_status.maximum_hp = maximum_hp
        self.robot_status.shooter_17mm_1_barrel_heat = heat
        self.robot_status.shooter_barrel_heat_limit = heat_limit
        self.robot_status.projectile_allowance_17mm = projectile_allowance_17mm
        self.robot_status.is_hp_deduced = is_hp_deduced
        self.robot_status.armor_id = armor_id
        self.robot_status.hp_deduction_reason = RobotStatus.ARMOR_HIT

    def set_rfid_status(
        self,
        friendly_supply_zone_non_exchange,
        friendly_supply_zone_exchange,
        center_gain_point,
        friendly_fortress_gain_point,
    ):
        self.rfid_status.friendly_supply_zone_non_exchange = (
            friendly_supply_zone_non_exchange
        )
        self.rfid_status.friendly_supply_zone_exchange = friendly_supply_zone_exchange
        self.rfid_status.center_gain_point = center_gain_point
        self.rfid_status.friendly_fortress_gain_point = friendly_fortress_gain_point

    def set_hp_status(self, red_outpost_hp, red_base_hp, blue_outpost_hp, blue_base_hp):
        self.game_robot_hp.red_outpost_hp = red_outpost_hp
        self.game_robot_hp.red_base_hp = red_base_hp
        self.game_robot_hp.blue_outpost_hp = blue_outpost_hp
        self.game_robot_hp.blue_base_hp = blue_base_hp

    def tick_countdown(self):
        is_running = self.game_status.game_progress == GameStatus.RUNNING
        if self.posture_simulation_enabled:
            self.posture_state.advance(1.0, is_running)
        if self.countdown_enabled and self.game_status.stage_remain_time > 0:
            self.game_status.stage_remain_time -= 1
        elif self.countdown_enabled and self.game_status.stage_remain_time == 0:
            self.game_status.game_progress = GameStatus.GAME_OVER

    def request_posture(self, posture):
        if not self.posture_simulation_enabled:
            return False, "posture simulation is disabled for this launch"
        is_running = self.game_status.game_progress == GameStatus.RUNNING
        accepted, message = self.posture_state.request(posture, is_running)
        log_message = "Sentry posture request {}: {}".format(posture, message)
        if accepted:
            self.get_logger().info(log_message)
        else:
            self.get_logger().warning(log_message)
        return accepted, message

    def _posture_command_callback(self, message):
        if self.manual_posture_override:
            if not self._manual_override_warning_emitted:
                self.get_logger().warning(
                    "Ignoring autonomous posture requests while manual override is enabled"
                )
                self._manual_override_warning_emitted = True
            return
        self.request_posture(message.posture)

    def publish_once(self):
        self.game_status_pub.publish(self.game_status)
        self.robot_status_pub.publish(self.robot_status)
        self.rfid_status_pub.publish(self.rfid_status)
        self.game_robot_hp_pub.publish(self.game_robot_hp)
        self.event_data_pub.publish(self.event_data)
        self.buff_pub.publish(self.buff)
        self.ground_robot_position_pub.publish(self.ground_robot_position)
        self.emergency_stop_pub.publish(self.emergency_stop)
        self.posture_state.fill_status_message(self.sentry_posture_status)
        self.sentry_posture_status_pub.publish(self.sentry_posture_status)
