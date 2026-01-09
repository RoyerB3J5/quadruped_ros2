import sys
import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel,
    QVBoxLayout, QHBoxLayout, QSlider, QComboBox,
    QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QGridLayout
from pathlib import Path
from robot_msgs.msg import MotionCommand
from std_msgs.msg import String


class TeachPendantNode(Node):
    def __init__(self):
        super().__init__('teach_pendant_gui')
        self.pub = self.create_publisher(MotionCommand, '/motion_command', 10)
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10
        )

        self.desired_command = None
        self.command = None
        self.gait = "trot"
        self.motion = "stop"
        self.velocity = 0.09
        self.step_height = 0.02
        self.step_length = 0.035
        self.z_height = 0.1
        self._initial_published = False

    def state_callback(self, msg):
        self.command = msg.data.lower()

    def publish(self):
        msg = MotionCommand()
        msg.command = self.desired_command
        msg.gait = self.gait
        msg.motion = self.motion
        msg.velocity = self.velocity
        msg.step_height = self.step_height
        msg.step_length = self.step_length
        msg.z_height = self.z_height
        self.pub.publish(msg)

    def publish_initial(self):
        if self._initial_published:
            return
        self.publish()
        self.get_logger().info('Initial MotionCommand published from GUI')
        self._initial_published = True


class TeachPendantGUI(QWidget):
    def __init__(self, node: TeachPendantNode):
        super().__init__()
        self.node = node

        self.setWindowTitle("Teach Pendant - Robot Cuadrúpedo")
        self.setMinimumWidth(1000)
        self.resize(1200, 700)

        self.setStyleSheet("""
        QWidget {
            background-color: white;
        }

        QWidget:disabled {
            color: #888;
        }

        QSlider:disabled {
            background-color: #ddd;
        }

        QPushButton:disabled {
            background-color: #aaa;
        }
        """)
        


        root = QVBoxLayout(self)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(15)


        title = QLabel("TEACH PENDANT – CUADRÚPEDO")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            font-size: 24px;
            font-weight: 600;
            color: white;
            background-color: #222;
            padding: 12px;
            border-radius: 10px;
        """)
        root.addWidget(title)

        self.state_label = QLabel("Estado: ACOSTADO")
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setStyleSheet("font-size:16px; font-weight:600;")
        root.addWidget(self.state_label)

        content = QHBoxLayout()
        content.setSpacing(30)

        left_widget = QWidget()                       
        left = QVBoxLayout(left_widget)
        left.setSpacing(14)

        left.addStretch(1)                            
        lbl_cmds = QLabel("COMANDOS")
        lbl_cmds.setAlignment(Qt.AlignCenter)
        lbl_cmds.setStyleSheet("font-weight:600;")
        left.addWidget(lbl_cmds, alignment=Qt.AlignCenter)

        btn_lie = self.make_button("ACOSTAR", "#c0392b")
        btn_lie.clicked.connect(lambda: self.set_command("lie"))

        btn_stand = self.make_button("PARAR", "#2980b9")
        btn_stand.clicked.connect(lambda: self.set_command("stand"))

        btn_walk = self.make_button("CAMINAR", "#27ae60")
        btn_walk.clicked.connect(lambda: self.set_command("walk"))

        left.addWidget(btn_lie)
        left.addWidget(btn_stand)
        left.addWidget(btn_walk)
        left.addStretch(1)                           

        lbl_move = QLabel("MOVIMIENTO")
        lbl_move.setAlignment(Qt.AlignCenter)
        lbl_move.setStyleSheet("font-weight:600;")
        left.addWidget(lbl_move, alignment=Qt.AlignCenter)

        grid = QGridLayout()
        grid.setSpacing(8)

        btn_up = self.make_button_2("↑", "#444")
        btn_up.setCheckable(True)
        btn_down = self.make_button_2("↓", "#444")
        btn_down.setCheckable(True)
        btn_left = self.make_button_2("←", "#444")
        btn_left.setCheckable(True)
        btn_right = self.make_button_2("→", "#444")
        btn_right.setCheckable(True)
        btn_stop = self.make_button_2("■", "#111")
        btn_stop.setCheckable(True)

        btn_up.clicked.connect(lambda: self.set_motion("forward"))
        btn_down.clicked.connect(lambda: self.set_motion("backward"))
        btn_left.clicked.connect(lambda: self.set_motion("left"))
        btn_right.clicked.connect(lambda: self.set_motion("right"))
        btn_stop.clicked.connect(lambda: self.set_motion("stop"))

        grid.addWidget(btn_up,    0, 1)
        grid.addWidget(btn_left,  1, 0)
        grid.addWidget(btn_stop,  1, 1)
        grid.addWidget(btn_right, 1, 2)
        grid.addWidget(btn_down,  2, 1)

        left.addLayout(grid)

        self.motion_buttons = {
            "forward": btn_up,
            "backward": btn_down,
            "left": btn_left,
            "right": btn_right,
            "stop": btn_stop,
        }

        self.allowed_motion_by_gait = {
            "trot": {"forward", "backward", "stop"},
            "crawl": {"forward", "backward", "left", "right", "stop"},
        }
        
        left.addStretch(1)
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding
        )

        img_path = Path(__file__).parent / 'images' / 'robot.png'
        pix = QPixmap(str(img_path))
        self.base_pixmap = pix if not pix.isNull() else None

        if not pix.isNull():
            self.image_label.setPixmap(
                pix.scaled(360, 360, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        else:
            self.image_label.setText("Imagen no encontrada")

        right_widget = QWidget()                     
        right = QVBoxLayout(right_widget)
        right.setSpacing(20)

        right.addStretch(1)                          

        layout, self.gait_combo = self.make_combo_box(
            "MARCHA",
            [("Caminata", "crawl"), ("Trote", "trot")],
            self.set_gait
        )
        right.addLayout(layout)

        # Set combo index to match node.gait without emitting the signal
        self.gait_combo.blockSignals(True)
        if self.node.gait == 'crawl':
            self.gait_combo.setCurrentIndex(0)
        else:
            self.gait_combo.setCurrentIndex(1)
        self.gait_combo.blockSignals(False)

        layout, self.z_height_slider = self.make_slider_box(
            "ALTURA ROBOT", 4, 17, int(self.node.z_height * 100),
            self.set_z_height, "cm", display_divisor=1
        )
        right.addLayout(layout)

        layout, self.velocity_slider = self.make_slider_box(
            "VELOCIDAD", 4, 20, int(self.node.velocity * 100),
            self.set_velocity, "m/s", display_divisor=100
        )
        right.addLayout(layout)

        layout, self.step_height_slider = self.make_slider_box(
            "ALTURA DEL PASO", 5, 40, int(self.node.step_height * 1000),
            self.set_step_height, "cm", display_divisor=10
        )
        right.addLayout(layout)

        right.addStretch(1)                           

        content.addWidget(left_widget, 3)
        content.addWidget(self.image_label, 4)
        content.addWidget(right_widget, 3)

        root.addLayout(content)

        self.setStyleSheet("background-color: white;")
        self.update_interblocks()
        self.adjustSize()
        self.setMinimumHeight(self.sizeHint().height())


        self.state_timer = QTimer()
        self.state_timer.timeout.connect(self.sync_state_label)
        self.state_timer.start(100)

    def sync_state_label(self):
        state_map = {
            "LIE": "ACOSTADO",
            "STAND": "PARADO",
            "WALK": "CAMINANDO",
            "TRANSITION_WALK_TO_STAND": "TRANSICIÓN A PARADO",
            "TRANSITION_STAND_TO_WALK": "TRANSICIÓN A CAMINANDO",
            "TRANSITION_LIE_TO_STAND": "TRANSICIÓN A PARADO",
            "TRANSITION_STAND_TO_LIE": "TRANSICIÓN A ACOSTADO",
        }

        cmd = self.node.command
        if cmd is None and self.node.desired_command is not None:
            cmd = self.node.desired_command

        text = "INICIADO" if cmd is None else state_map.get(cmd.upper(), cmd.upper())
        self.state_label.setText(f"Estado: {text}")
        self.update_interblocks()


    def make_button(self, text, color):
        btn = QPushButton(text)
        btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
                color: white;
                font-size: 14px;
                padding: 12px;
                border-radius: 8px;
            }}
            QPushButton:hover {{
                background-color: #555;
            }}
        """)
        return btn
    
    def make_button_2(self, text, color):
        btn = QPushButton(text)
        btn.setCheckable(True)
        btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        base_style = f"""
        QPushButton {{
            background-color: {color};
            color: white;
            font-size: 16px;
            padding: 12px;
            border-radius: 8px;
        }}
        """
        disabled_style = """
        QPushButton {
            background-color: #cccccc;
            color: #777;
            font-size: 16px;
            padding: 12px;
            border-radius: 8px;
        }
        """
        btn.setStyleSheet(base_style)
        btn.setProperty("base_style", base_style)
        btn.setProperty("disabled_style", disabled_style)
        return btn
    
    def active_style(self, color):
        return f"""
            QPushButton {{
                background-color: {color};
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding:12px;
                border-radius: 8px;
            }}
        """

    def update_motion_buttons_by_gait(self):
        cmd = self.node.desired_command
        real_cmd = self.node.command
        in_transition = (real_cmd is not None) and ("transition" in real_cmd)

        enabled_base = (cmd == "walk") and not in_transition

        allowed = self.allowed_motion_by_gait[self.node.gait]

        for motion, btn in self.motion_buttons.items():
            enabled = enabled_base and (motion in allowed)
            btn.setEnabled(enabled)

            if not enabled:
                btn.setChecked(False)
                btn.setStyleSheet(btn.property("disabled_style"))
            else:
                # If this motion matches the current node.motion, mark it active
                if self.node.motion == motion:
                    btn.setChecked(True)
                    color = "#c0392b" if motion == "stop" else "#27ae60"
                    btn.setStyleSheet(self.active_style(color))
                else:
                    btn.setChecked(False)
                    btn.setStyleSheet(btn.property("base_style"))

        # If the interface is enabling motion controls but the current motion
        # is not allowed for the selected gait, ensure we stay in 'stop'.
        if enabled_base and self.node.motion not in allowed:
            # Use set_motion so styles and checked states update consistently
            self.set_motion("stop")


    def make_combo_box(self, title, items, callback):
        box = QVBoxLayout()
        title_lbl = QLabel(title)
        title_lbl.setAlignment(Qt.AlignCenter)
        title_lbl.setStyleSheet("font-weight:600;")
        box.addWidget(title_lbl, alignment=Qt.AlignCenter)

        combo = QComboBox()
        for label, value in items:
            combo.addItem(label, value)

        combo.currentIndexChanged.connect(
            lambda i: callback(combo.itemData(i))
        )
        combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        box.addWidget(combo)
        return box, combo

    def make_slider_box(self, title, min_v, max_v, value, callback, unit, display_divisor=None):
        box = QVBoxLayout()

        title_lbl = QLabel(title)
        title_lbl.setAlignment(Qt.AlignCenter)
        title_lbl.setStyleSheet("font-weight:600;")
        box.addWidget(title_lbl, alignment=Qt.AlignCenter)

        slider = QSlider(Qt.Horizontal)
        slider.setRange(min_v, max_v)
        slider.setValue(value)
        slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        label = QLabel()
        label.setAlignment(Qt.AlignCenter)

        def preview(v):
            if display_divisor:
                val = v / display_divisor
                label.setText(f"{val:.2f} {unit}")
            else:
                label.setText(f"{v} {unit}")

        slider.valueChanged.connect(preview)

        slider.sliderReleased.connect(lambda: callback(slider.value()))

        preview(value)

        box.addWidget(slider)
        box.addWidget(label)
        return box, slider

    def update_interblocks(self):
        cmd = self.node.command
        if cmd is None and self.node.desired_command is not None:
            cmd = self.node.desired_command

        self.z_height_slider.setEnabled(cmd in ["stand", "lie"])
        self.velocity_slider.setEnabled(cmd in ["walk", "stand"])
        self.step_height_slider.setEnabled(cmd in ["walk", "stand"])
        self.gait_combo.setEnabled(cmd in ["walk", "stand"])
        
        self.update_motion_buttons_by_gait()


    def set_motion(self, motion):
        if self.node.command not in [None, "walk"] and motion != "stop":
            motion = "stop"

        if self.node.motion == motion:
            return

        if self.node.motion in self.motion_buttons:
            prev = self.motion_buttons[self.node.motion]
            prev.setChecked(False)
            prev.setStyleSheet(prev.property("base_style"))

        btn = self.motion_buttons[motion]
        btn.setChecked(True)

        if motion == "stop":
            btn.setStyleSheet(self.active_style("#c0392b"))
            QTimer.singleShot(
                3000,
                lambda b=btn: (
                    b.setStyleSheet(b.property("base_style"))
                    if self.node.motion == "stop" else None
                )
            )
        else:
            btn.setStyleSheet(self.active_style("#27ae60"))

        self.node.motion = motion
        self.node.publish()



    def set_command(self, cmd):
        self.node.desired_command = cmd

        if cmd in ["lie", "stand"]:
            self.node.motion = "stop"

        self.node.publish()
        self.update_interblocks()


    def set_gait(self, gait):
        self.node.gait = gait
        self.update_motion_buttons_by_gait()
        self.node.publish()

    def set_z_height(self, v):
        self.node.z_height = v / 100
        self.node.publish()

    def set_velocity(self, v):
        self.node.velocity = v / 100
        self.node.publish()

    def set_step_height(self, v):
        self.node.step_height = v / 1000
        self.node.publish()

    def resizeEvent(self, event):
        if self.base_pixmap:
            w = self.image_label.width()
            self.image_label.setPixmap(
                self.base_pixmap.scaledToWidth(w, Qt.SmoothTransformation)
            )
        super().resizeEvent(event)


def main():
    rclpy.init()
    node = TeachPendantNode()

    app = QApplication(sys.argv)
    gui = TeachPendantGUI(node)
    gui.show()

    def spin_ros():
        rclpy.spin_once(node, timeout_sec=0.0)

    ros_timer = QTimer()
    ros_timer.timeout.connect(spin_ros)
    ros_timer.start(20)

    app.exec_()

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
