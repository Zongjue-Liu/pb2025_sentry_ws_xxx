import signal
import sys

import rclpy

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QCheckBox
from python_qt_binding.QtWidgets import QComboBox
from python_qt_binding.QtWidgets import QFormLayout
from python_qt_binding.QtWidgets import QGroupBox
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QScrollArea
from python_qt_binding.QtWidgets import QSpinBox
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget
from std_srvs.srv import Trigger

from pb_control_panel.publisher import PbControlPanelPublisher


GAME_PROGRESS_ITEMS = [
    ("0 NOT_START", 0),
    ("1 PREPARATION", 1),
    ("2 SELF_CHECKING", 2),
    ("3 COUNT_DOWN", 3),
    ("4 RUNNING", 4),
    ("5 GAME_OVER", 5),
]


class PbControlPanelGui(QMainWindow):
    PREPARATION_DELAY_MS = 500
    SELF_CHECKING_DELAY_MS = 1000
    COUNTDOWN_DELAY_MS = 1000

    def __init__(self, publisher: PbControlPanelPublisher):
        super().__init__()
        self.publisher = publisher
        self.setWindowTitle("PB Control Panel")

        self.game_progress = self._combo_box(GAME_PROGRESS_ITEMS, current_value=0)
        self.stage_remain_time = self._spin_box(0, 32767, self.publisher.match_duration)
        self.countdown_enabled = QCheckBox("countdown")
        self.countdown_enabled.setChecked(False)
        self.emergency_stop = QCheckBox("emergency stop")

        self.current_hp = self._spin_box(0, 2000, 400)
        self.maximum_hp = self._spin_box(0, 2000, 400)
        self.heat = self._spin_box(0, 2000, 0)
        self.heat_limit = self._spin_box(0, 2000, 260)
        self.projectile_allowance_17mm = self._spin_box(0, 2000, 750)
        self.is_hp_deduced = QCheckBox("armor hit")
        self.armor_id = self._spin_box(0, 10, 0)

        self.supply_non_exchange = QCheckBox("supply non exchange")
        self.supply_exchange = QCheckBox("supply exchange")
        self.center_gain_point = QCheckBox("center gain point")
        self.fortress_gain_point = QCheckBox("fortress gain point")

        self.red_outpost_hp = self._spin_box(0, 2000, 1000)
        self.red_base_hp = self._spin_box(0, 5000, 1000)
        self.blue_outpost_hp = self._spin_box(0, 2000, 1000)
        self.blue_base_hp = self._spin_box(0, 5000, 1000)

        self.status_label = QLabel("not published yet")
        self.start_button = QPushButton("start new match")
        self.start_button.clicked.connect(self.start_match)
        self.end_button = QPushButton("end match")
        self.end_button.clicked.connect(self.end_match)
        self.apply_button = QPushButton("apply and publish")
        self.apply_button.clicked.connect(self.apply_to_publisher)

        match_button_layout = QHBoxLayout()
        match_button_layout.addWidget(self.start_button)
        match_button_layout.addWidget(self.end_button)

        form_layout = QVBoxLayout()
        form_layout.addWidget(self._game_group())
        form_layout.addWidget(self._robot_group())
        form_layout.addWidget(self._rfid_group())
        form_layout.addWidget(self._hp_group())

        form_widget = QWidget()
        form_widget.setLayout(form_layout)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(form_widget)

        root_layout = QVBoxLayout()
        root_layout.addWidget(scroll_area)
        root_layout.addLayout(match_button_layout)
        root_layout.addWidget(self.apply_button)
        root_layout.addWidget(self.status_label)

        central_widget = QWidget()
        central_widget.setLayout(root_layout)
        self.setCentralWidget(central_widget)
        self.resize(520, 640)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(1000)

        self.match_phase_timer = QTimer(self)
        self.match_phase_timer.setSingleShot(True)
        self.match_phase_timer.timeout.connect(self._advance_match_start)
        self.match_start_phase = None

        self.start_match_service = self.publisher.create_service(
            Trigger, "referee/start_new_match", self._handle_start_match_service
        )
        self.end_match_service = self.publisher.create_service(
            Trigger, "referee/end_match", self._handle_end_match_service
        )

        self.apply_to_publisher()

    def _combo_box(self, items, current_value):
        combo_box = QComboBox()
        for text, value in items:
            combo_box.addItem(text, value)
            if value == current_value:
                combo_box.setCurrentIndex(combo_box.count() - 1)
        return combo_box

    def _spin_box(self, minimum, maximum, value):
        spin_box = QSpinBox()
        spin_box.setRange(minimum, maximum)
        spin_box.setValue(value)
        return spin_box

    def _game_group(self):
        group = QGroupBox("game_status")
        layout = QFormLayout()
        layout.addRow("game_progress", self.game_progress)
        layout.addRow("stage_remain_time", self.stage_remain_time)
        layout.addRow("", self.countdown_enabled)
        layout.addRow("", self.emergency_stop)
        group.setLayout(layout)
        return group

    def _robot_group(self):
        group = QGroupBox("robot_status")
        layout = QFormLayout()
        layout.addRow("current_hp", self.current_hp)
        layout.addRow("maximum_hp", self.maximum_hp)
        layout.addRow("heat", self.heat)
        layout.addRow("heat_limit", self.heat_limit)
        layout.addRow("projectile_17mm", self.projectile_allowance_17mm)

        hit_layout = QHBoxLayout()
        hit_layout.addWidget(self.is_hp_deduced)
        hit_layout.addWidget(QLabel("armor_id"))
        hit_layout.addWidget(self.armor_id)
        layout.addRow("hit test", hit_layout)

        group.setLayout(layout)
        return group

    def _rfid_group(self):
        group = QGroupBox("rfid_status")
        layout = QVBoxLayout()
        layout.addWidget(self.supply_non_exchange)
        layout.addWidget(self.supply_exchange)
        layout.addWidget(self.center_gain_point)
        layout.addWidget(self.fortress_gain_point)
        group.setLayout(layout)
        return group

    def _hp_group(self):
        group = QGroupBox("all_robot_hp")
        layout = QFormLayout()
        layout.addRow("red_outpost_hp", self.red_outpost_hp)
        layout.addRow("red_base_hp", self.red_base_hp)
        layout.addRow("blue_outpost_hp", self.blue_outpost_hp)
        layout.addRow("blue_base_hp", self.blue_base_hp)
        group.setLayout(layout)
        return group

    def apply_to_publisher(self):
        self.publisher.set_game_status(
            self.game_progress.currentData(),
            self.stage_remain_time.value(),
        )
        self.publisher.countdown_enabled = self.countdown_enabled.isChecked()
        self.publisher.set_emergency_stop(self.emergency_stop.isChecked())
        self.publisher.set_robot_status(
            self.current_hp.value(),
            self.maximum_hp.value(),
            self.heat.value(),
            self.heat_limit.value(),
            self.projectile_allowance_17mm.value(),
            self.is_hp_deduced.isChecked(),
            self.armor_id.value(),
        )
        self.publisher.set_rfid_status(
            self.supply_non_exchange.isChecked(),
            self.supply_exchange.isChecked(),
            self.center_gain_point.isChecked(),
            self.fortress_gain_point.isChecked(),
        )
        self.publisher.set_hp_status(
            self.red_outpost_hp.value(),
            self.red_base_hp.value(),
            self.blue_outpost_hp.value(),
            self.blue_base_hp.value(),
        )
        self.publisher.publish_once()
        self.status_label.setText("published")

    def start_match(self):
        if self.match_start_phase is not None:
            return
        self.publisher.set_emergency_stop(self.emergency_stop.isChecked())
        self.publisher.start_preparation()
        self.match_start_phase = "preparation"
        self.start_button.setEnabled(False)
        self._sync_game_widgets()
        self.publisher.publish_once()
        self.status_label.setText("resetting robots: preparation")
        self.match_phase_timer.start(self.PREPARATION_DELAY_MS)

    def _handle_start_match_service(self, request, response):
        del request
        if self.match_start_phase is not None:
            response.success = False
            response.message = "match start sequence is already running"
            return response

        self.start_match()
        response.success = True
        response.message = "match start sequence accepted"
        return response

    def _advance_match_start(self):
        self.match_phase_timer.stop()
        if self.match_start_phase == "preparation":
            self.publisher.start_self_checking()
            self.match_start_phase = "self_checking"
            delay = self.SELF_CHECKING_DELAY_MS
        elif self.match_start_phase == "self_checking":
            self.publisher.start_countdown()
            self.match_start_phase = "countdown"
            delay = self.COUNTDOWN_DELAY_MS
        elif self.match_start_phase == "countdown":
            self.publisher.start_match()
            self.match_start_phase = None
            self.start_button.setEnabled(True)
            delay = None
        else:
            return

        self._sync_game_widgets()
        self.publisher.publish_once()
        if delay is not None:
            self.match_phase_timer.start(delay)

    def end_match(self):
        self.match_phase_timer.stop()
        self.match_start_phase = None
        self.start_button.setEnabled(True)
        self.publisher.end_match()
        self._sync_game_widgets()
        self.publisher.publish_once()
        self.status_label.setText("match ended")

    def _handle_end_match_service(self, request, response):
        del request
        self.end_match()
        response.success = True
        response.message = "match ended"
        return response

    def _sync_game_widgets(self):
        game_progress = self.publisher.game_status.game_progress
        index = self.game_progress.findData(game_progress)
        if index >= 0:
            self.game_progress.setCurrentIndex(index)
        self.stage_remain_time.setValue(self.publisher.game_status.stage_remain_time)
        self.countdown_enabled.setChecked(self.publisher.countdown_enabled)

    def on_timer(self):
        rclpy.spin_once(self.publisher, timeout_sec=0.0)
        self.publisher.tick_countdown()
        self._sync_game_widgets()
        self.publisher.publish_once()
        self.status_label.setText(
            "publishing: game_progress={} remain_time={}".format(
                self.publisher.game_status.game_progress,
                self.publisher.game_status.stage_remain_time,
            )
        )

    def closeEvent(self, event):
        self.timer.stop()
        self.match_phase_timer.stop()
        self.publisher.end_match()
        self.publisher.publish_once()
        self.publisher.destroy_node()
        rclpy.shutdown()
        event.accept()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    panel_pub = PbControlPanelPublisher()
    panel = PbControlPanelGui(panel_pub)
    panel.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
