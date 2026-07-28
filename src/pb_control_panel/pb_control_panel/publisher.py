import rclpy
from rclpy.node import Node

from pb_rm_interfaces.msg import Buff
from pb_rm_interfaces.msg import EventData
from pb_rm_interfaces.msg import GameRobotHP
from pb_rm_interfaces.msg import GameStatus
from pb_rm_interfaces.msg import GroundRobotPosition
from pb_rm_interfaces.msg import RfidStatus
from pb_rm_interfaces.msg import RobotStatus


class PbControlPanelPublisher(Node):
    def __init__(self, node_name='pb_control_panel'):
        super().__init__(node_name)

        self.game_status_pub = self.create_publisher(GameStatus, 'referee/game_status', 10)
        self.robot_status_pub = self.create_publisher(RobotStatus, 'referee/robot_status', 10)
        self.rfid_status_pub = self.create_publisher(RfidStatus, 'referee/rfid_status', 10)
        self.game_robot_hp_pub = self.create_publisher(GameRobotHP, 'referee/all_robot_hp', 10)
        self.event_data_pub = self.create_publisher(EventData, 'referee/event_data', 10)
        self.buff_pub = self.create_publisher(Buff, 'referee/buff', 10)
        self.ground_robot_position_pub = self.create_publisher(
            GroundRobotPosition, 'referee/ground_robot_position', 10
        )

        self.game_status = GameStatus()
        self.game_status.game_progress = GameStatus.RUNNING
        self.game_status.stage_remain_time = 420

        self.robot_status = RobotStatus()
        self.robot_status.robot_id = 7
        self.robot_status.robot_level = 1
        self.robot_status.current_hp = 400
        self.robot_status.maximum_hp = 400
        self.robot_status.shooter_barrel_cooling_value = 10
        self.robot_status.shooter_barrel_heat_limit = 400
        self.robot_status.shooter_17mm_1_barrel_heat = 0
        self.robot_status.armor_id = 0
        self.robot_status.hp_deduction_reason = RobotStatus.ARMOR_HIT
        self.robot_status.projectile_allowance_17mm = 100
        self.robot_status.remaining_gold_coin = 0
        self.robot_status.is_hp_deduced = False

        self.rfid_status = RfidStatus()
        self.game_robot_hp = GameRobotHP()
        self.event_data = EventData()
        self.buff = Buff()
        self.ground_robot_position = GroundRobotPosition()

        self.countdown_enabled = True

    def set_game_status(self, game_progress, stage_remain_time):
        self.game_status.game_progress = game_progress
        self.game_status.stage_remain_time = stage_remain_time

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
        self.rfid_status.friendly_supply_zone_non_exchange = friendly_supply_zone_non_exchange
        self.rfid_status.friendly_supply_zone_exchange = friendly_supply_zone_exchange
        self.rfid_status.center_gain_point = center_gain_point
        self.rfid_status.friendly_fortress_gain_point = friendly_fortress_gain_point

    def set_hp_status(self, red_outpost_hp, red_base_hp, blue_outpost_hp, blue_base_hp):
        self.game_robot_hp.red_outpost_hp = red_outpost_hp
        self.game_robot_hp.red_base_hp = red_base_hp
        self.game_robot_hp.blue_outpost_hp = blue_outpost_hp
        self.game_robot_hp.blue_base_hp = blue_base_hp

    def tick_countdown(self):
        if self.countdown_enabled and self.game_status.stage_remain_time > 0:
            self.game_status.stage_remain_time -= 1
        elif self.countdown_enabled and self.game_status.stage_remain_time == 0:
            self.game_status.game_progress = GameStatus.GAME_OVER

    def publish_once(self):
        self.game_status_pub.publish(self.game_status)
        self.robot_status_pub.publish(self.robot_status)
        self.rfid_status_pub.publish(self.rfid_status)
        self.game_robot_hp_pub.publish(self.game_robot_hp)
        self.event_data_pub.publish(self.event_data)
        self.buff_pub.publish(self.buff)
        self.ground_robot_position_pub.publish(self.ground_robot_position)
