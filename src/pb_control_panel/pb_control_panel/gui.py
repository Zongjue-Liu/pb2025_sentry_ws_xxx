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

from pb_control_panel.publisher import PbControlPanelPublisher


GAME_PROGRESS_ITEMS = [
    ('0 NOT_START', 0),
    ('1 PREPARATION', 1),
    ('2 SELF_CHECKING', 2),
    ('3 COUNT_DOWN', 3),
    ('4 RUNNING', 4),
    ('5 GAME_OVER', 5),
]

class PbControlPanelGui(QMainWindow):
    def __init__(self, publisher: PbControlPanelPublisher):
        super().__init__()
        self.publisher = publisher
        self.setWindowTitle('PB Control Panel')

        self.game_progress = self._combo_box(GAME_PROGRESS_ITEMS, current_value=4)
        self.stage_remain_time = self._spin_box(0, 32767, 420)
        self.countdown_enabled = QCheckBox('countdown')
        self.countdown_enabled.setChecked(True)

        self.current_hp = self._spin_box(0, 2000, 400)
        self.maximum_hp = self._spin_box(0, 2000, 400)
        self.heat = self._spin_box(0, 2000, 0)
        self.heat_limit = self._spin_box(0, 2000, 400)
        self.projectile_allowance_17mm = self._spin_box(0, 2000, 100)
        self.is_hp_deduced = QCheckBox('armor hit')
        self.armor_id = self._spin_box(0, 10, 0)

        self.supply_non_exchange = QCheckBox('supply non exchange')
        self.supply_exchange = QCheckBox('supply exchange')
        self.center_gain_point = QCheckBox('center gain point')
        self.fortress_gain_point = QCheckBox('fortress gain point')

        self.red_outpost_hp = self._spin_box(0, 2000, 1000)
        self.red_base_hp = self._spin_box(0, 5000, 1000)
        self.blue_outpost_hp = self._spin_box(0, 2000, 1000)
        self.blue_base_hp = self._spin_box(0, 5000, 1000)

        self.status_label = QLabel('not published yet')
        self.apply_button = QPushButton('apply and publish')
        self.apply_button.clicked.connect(self.apply_to_publisher)

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
        root_layout.addWidget(self.apply_button)
        root_layout.addWidget(self.status_label)

        central_widget = QWidget()
        central_widget.setLayout(root_layout)
        self.setCentralWidget(central_widget)
        self.resize(520, 640)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(1000)

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
        group = QGroupBox('game_status')
        layout = QFormLayout()
        layout.addRow('game_progress', self.game_progress)
        layout.addRow('stage_remain_time', self.stage_remain_time)
        layout.addRow('', self.countdown_enabled)
        group.setLayout(layout)
        return group

    def _robot_group(self):
        group = QGroupBox('robot_status')
        layout = QFormLayout()
        layout.addRow('current_hp', self.current_hp)
        layout.addRow('maximum_hp', self.maximum_hp)
        layout.addRow('heat', self.heat)
        layout.addRow('heat_limit', self.heat_limit)
        layout.addRow('projectile_17mm', self.projectile_allowance_17mm)

        hit_layout = QHBoxLayout()
        hit_layout.addWidget(self.is_hp_deduced)
        hit_layout.addWidget(QLabel('armor_id'))
        hit_layout.addWidget(self.armor_id)
        layout.addRow('hit test', hit_layout)

        group.setLayout(layout)
        return group

    def _rfid_group(self):
        group = QGroupBox('rfid_status')
        layout = QVBoxLayout()
        layout.addWidget(self.supply_non_exchange)
        layout.addWidget(self.supply_exchange)
        layout.addWidget(self.center_gain_point)
        layout.addWidget(self.fortress_gain_point)
        group.setLayout(layout)
        return group

    def _hp_group(self):
        group = QGroupBox('all_robot_hp')
        layout = QFormLayout()
        layout.addRow('red_outpost_hp', self.red_outpost_hp)
        layout.addRow('red_base_hp', self.red_base_hp)
        layout.addRow('blue_outpost_hp', self.blue_outpost_hp)
        layout.addRow('blue_base_hp', self.blue_base_hp)
        group.setLayout(layout)
        return group

    def apply_to_publisher(self):
        self.publisher.set_game_status(
            self.game_progress.currentData(),
            self.stage_remain_time.value(),
        )
        self.publisher.countdown_enabled = self.countdown_enabled.isChecked()
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
        self.status_label.setText('published')

    def on_timer(self):
        rclpy.spin_once(self.publisher, timeout_sec=0.0)
        self.publisher.tick_countdown()
        self.stage_remain_time.setValue(self.publisher.game_status.stage_remain_time)
        self.publisher.publish_once()
        self.status_label.setText(
            'publishing: game_progress={} remain_time={}'.format(
                self.publisher.game_status.game_progress,
                self.publisher.game_status.stage_remain_time,
            )
        )

    def closeEvent(self, event):
        self.timer.stop()
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


if __name__ == '__main__':
    main()
