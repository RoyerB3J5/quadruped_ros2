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


class TeachPendantNode(Node):
    def __init__(self):
        super().__init__('teach_pendant_gui')
        self.pub = self.create_publisher(MotionCommand, '/motion_command', 10)

        self.command = "lie"
        self.gait = "trot"
        self.motion = "stop"
        self.velocity = 0.09
        self.step_height = 0.01
        self.step_length = 0.035
        self.z_height = 0.1

    def publish(self):
        msg = MotionCommand()
        msg.command = self.command
        msg.gait = self.gait
        msg.motion = self.motion
        msg.velocity = self.velocity
        msg.step_height = self.step_height
        msg.step_length = self.step_length
        msg.z_height = self.z_height
        self.pub.publish(msg)


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

        # map motion names to the button objects (after buttons created)
        self.motion_buttons = {
            "forward": btn_up,
            "backward": btn_down,
            "left": btn_left,
            "right": btn_right,
            "stop": btn_stop,
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

        layout, self.z_height_slider = self.make_slider_box(
            "ALTURA ROBOT", 3, 17, int(self.node.z_height * 100),
            self.set_z_height, "cm", display_divisor=1
        )
        right.addLayout(layout)

        layout, self.velocity_slider = self.make_slider_box(
            "VELOCIDAD", 3, 20, int(self.node.velocity * 100),
            self.set_velocity, "m/s", display_divisor=100
        )
        right.addLayout(layout)

        layout, self.step_height_slider = self.make_slider_box(
            "ALTURA DEL PASO", 5, 30, int(self.node.step_height * 1000),
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

        QTimer.singleShot(200, self.node.publish)

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
        motion_enabled = (cmd == "walk")

        for btn in self.motion_buttons.values():
            btn.setEnabled(motion_enabled)

            if motion_enabled:
                btn.setStyleSheet(btn.property("base_style"))
            else:
                btn.setStyleSheet(btn.property("disabled_style"))
                btn.setChecked(False)
        
        self.z_height_slider.setEnabled(cmd in ["stand", "lie"])

        self.velocity_slider.setEnabled(cmd in ["walk", "stand"])

        self.step_height_slider.setEnabled(cmd in ["walk", "stand"])

        self.gait_combo.setEnabled(cmd in ["walk", "stand"])

    def set_motion(self, motion):
        if self.node.command != "walk":
            self.node.motion = "stop"
        for btn in self.motion_buttons.values():
            btn.setChecked(False)
            btn.setStyleSheet(btn.property("base_style"))

        if motion in self.motion_buttons:
            btn = self.motion_buttons[motion]
            btn.setChecked(True)

            if motion == "stop":
                btn.setStyleSheet(self.active_style("#c0392b"))
                QTimer.singleShot(3000, lambda b=btn: b.setStyleSheet(b.property("base_style")))
            else:
                btn.setStyleSheet(self.active_style("#27ae60"))

        self.node.motion = motion
        self.node.publish()

    def set_command(self, cmd):
        self.node.command = cmd
        self.state_label.setText(f"Estado: {cmd.upper()}")

        if cmd != "walk":
            self.node.motion = "stop"

        self.update_interblocks()
        self.node.publish()

    def set_gait(self, gait):
        self.node.gait = gait
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

    timer = QTimer()
    timer.start(50)

    app.exec_()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
