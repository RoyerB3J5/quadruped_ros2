import sys
import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel,
    QVBoxLayout, QHBoxLayout, QSlider, QComboBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QSizePolicy
from robot_msgs.msg import MotionCommand


class TeachPendantNode(Node):
    def __init__(self):
        super().__init__('teach_pendant_gui')
        self.pub = self.create_publisher(MotionCommand, '/motion_command', 10)

        # Estado (igual que tu código de teclado)
        self.command = "lie"
        self.gait = "trot"
        self.velocity = 0.15
        self.step_height = 0.01
        self.step_length = 0.035
        self.z_height = 0.1 

    def publish(self):
        msg = MotionCommand()
        msg.command = self.command
        msg.gait = self.gait
        msg.velocity = self.velocity
        msg.step_height = self.step_height
        msg.step_length = self.step_length

        self.pub.publish(msg)
        self.get_logger().info(
            f"[GUI] {msg.command} | {msg.gait} | v={msg.velocity:.2f} h={msg.step_height:.3f}"
        )


class TeachPendantGUI(QWidget):
    def __init__(self, node: TeachPendantNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("Teach Pendant - Robot Cuadrúpedo")
        self.setMinimumSize(700,600)
        self.resize(800,700)

        root = QVBoxLayout()
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(15)

        title = QLabel("TEACH PENDANT – CUADRÚPEDO")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: white;
            background-color: #222;
            padding: 12px;
            border-radius: 10px;
        """)
        root.addWidget(title)


        self.state_label = QLabel("Estado: ACOSTADO")
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setStyleSheet("""
            font-size: 16px;
            font-weight: bold;
            padding: 8px;
        """)
        root.addWidget(self.state_label)

        content = QHBoxLayout()
        content.setSpacing(30)
        
        left = QVBoxLayout()
        left.setSpacing(12)
        left.addWidget(QLabel("COMANDOS"), alignment=Qt.AlignCenter)
        left.setAlignment(Qt.AlignTop | Qt.AlignHCenter)

        btn_lie = self.make_button("ACOSTAR", "#e74c3c")
        btn_lie.clicked.connect(lambda: self.set_command("lie"))

        btn_stand = self.make_button("PARAR", "#3498db")
        btn_stand.clicked.connect(lambda: self.set_command("stand"))

        btn_walk = self.make_button("CAMINAR", "#2ecc71")
        btn_walk.clicked.connect(lambda: self.set_command("walk"))

        left.addWidget(btn_lie)
        left.addWidget(btn_stand)
        left.addWidget(btn_walk)
        
        left.addStretch()

        # Columna derecha
        right = QVBoxLayout()
        right.setSpacing(20)
        right.setAlignment(Qt.AlignTop | Qt.AlignHCenter)

        # Tipo de Marcha
        marcha_box = QVBoxLayout()
        marcha_box.setAlignment(Qt.AlignCenter)
        marcha_box.setSpacing(6)

        marcha_box.addWidget(QLabel("MARCHA"), alignment=Qt.AlignCenter)
        self.gait_box = QComboBox()
        self.gait_box.addItem("Caminata", "crawl")
        self.gait_box.addItem("Trote", "trot")
        self.gait_box.currentIndexChanged.connect(
            lambda i: self.set_gait(self.gait_box.itemData(i))
        )
        self.gait_box.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Fixed
        )
        marcha_box.addWidget(self.gait_box)

        right.addLayout(marcha_box)

        # Altura Robot
        robot_h_box = QVBoxLayout()
        robot_h_box.setAlignment(Qt.AlignCenter)
        robot_h_box.setSpacing(6)
        robot_h_box.addWidget(QLabel("ALTURA ROBOT"), alignment=Qt.AlignCenter)

        self.robot_h_slider = QSlider(Qt.Horizontal)
        self.robot_h_slider.setRange(3, 17)  
        self.robot_h_slider.setValue(int(self.node.z_height * 100))
        self.robot_h_slider.valueChanged.connect(self.set_z_height)
        self.robot_h_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        
        self.z_height_value = QLabel(f"{self.node.z_height * 100:.1f} cm")
        self.z_height_value.setAlignment(Qt.AlignCenter)

        robot_h_box.addWidget(self.robot_h_slider)
        robot_h_box.addWidget(self.z_height_value)

        right.addLayout(robot_h_box)

        #Velocidad
        vel_box = QVBoxLayout()
        vel_box.setAlignment(Qt.AlignCenter)
        vel_box.setSpacing(6)
        vel_box.addWidget(QLabel("VELOCIDAD"), alignment=Qt.AlignCenter)

        self.vel_slider = QSlider(Qt.Horizontal)
        self.vel_slider.setRange(5, 40)  
        self.vel_slider.setValue(int(self.node.velocity * 100))
        self.vel_slider.valueChanged.connect(self.set_velocity)
        self.vel_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        
        self.vel_value = QLabel(f"{self.node.velocity:.2f} m/s")
        self.vel_value.setAlignment(Qt.AlignCenter)

        vel_box.addWidget(self.vel_slider)
        vel_box.addWidget(self.vel_value)

        right.addLayout(vel_box)

        #Altura de Paso
        h_box = QVBoxLayout()
        h_box.setAlignment(Qt.AlignCenter)
        h_box.setSpacing(6)
        
        h_box.addWidget(QLabel("ALTURA DEL PASO"), alignment=Qt.AlignCenter)
        self.h_slider = QSlider(Qt.Horizontal)
        self.h_slider.setRange(5, 30)  
        self.h_slider.setValue(int(self.node.step_height * 1000))
        self.h_slider.valueChanged.connect(self.set_step_height)
        self.h_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.h_value = QLabel(f"{self.node.step_height * 100:.1f} cm")
        self.h_value.setAlignment(Qt.AlignCenter)

        h_box.addWidget(self.h_slider)
        h_box.addWidget(self.h_value)

        right.addLayout(h_box)

        right.addStretch()

        content.addLayout(left,1)
        content.addLayout(right,1)

        root.addLayout(content)
        self.setLayout(root)

    def set_command(self, cmd):
        self.node.command = cmd
        self.state_label.setText(f"Estado: {cmd.upper()}")
        self.node.publish()

    def set_gait(self, gait):
        self.node.gait = gait
        self.node.publish()

    def set_z_height(self, value):
        self.node.z_height = value / 100.0
        self.z_height_value.setText(f"{self.node.z_height * 100:.1f} cm")
        self.node.publish()

    def set_velocity(self, value):
        self.node.velocity = value / 100.0
        self.vel_value.setText(f"{self.node.velocity:.2f} m/s")
        self.node.publish()


    def set_step_height(self, value):
        self.node.step_height = value / 1000.0
        self.h_value.setText(f"{self.node.step_height * 100:.1f} cm")
        self.node.publish()
    
    def make_button(self, text, color):
        btn = QPushButton(text)
        
        btn.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Fixed
        )
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
                color: white;
                font-size: 14px;
                padding: 10px;
                border-radius: 8px;
            }}
            QPushButton:hover {{
                background-color: #555;
            }}
        """)
        return btn



def main():
    rclpy.init()

    node = TeachPendantNode()

    app = QApplication(sys.argv)
    gui = TeachPendantGUI(node)
    gui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(50)

    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
