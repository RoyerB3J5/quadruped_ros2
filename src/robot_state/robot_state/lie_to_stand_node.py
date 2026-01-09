import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import numpy as np
from robot_msgs.msg import MotionCommand

class LieToStandNode(Node):
    def __init__(self):
        super().__init__('lie_to_stand_node')

        self.pub_fl = self.create_publisher(Point, '/foot_ref_transition/front_left', 10)
        self.pub_fr = self.create_publisher(Point, '/foot_ref_transition/front_right', 10)
        self.pub_rl = self.create_publisher(Point, '/foot_ref_transition/rear_left', 10)
        self.pub_rr = self.create_publisher(Point, '/foot_ref_transition/rear_right', 10)

        self.done_pub = self.create_publisher(String, '/transition_done', 10)

        self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            10
        )

        self.cmd_sub = self.create_subscription(
            MotionCommand,
            '/motion_command',
            self.command_callback,
            10
        )

        self.target_z = 0.10

        self.dt = 0.05
        self.duration = 3.0
        self.total_steps = max(2, int(self.duration / self.dt))

        self.step = 0
        self.active = False

        self.timer = self.create_timer(self.dt, self.update)

        self.x = 0.0
        self.y_left = 0.04005
        self.y_right = -0.04005

        self.z_lie = -0.04
        self.z_stand = -0.10

        self.get_logger().info('LieToStandNode listo')

    def command_callback(self, msg):
        self.target_z = msg.z_height

    def state_callback(self, msg):
        if msg.data == 'TRANSITION_LIE_TO_STAND' and not self.active:
            self.active = True
            self.step = 0
            self.z_stand = -self.target_z
            self.z_lie = -0.04
            self.get_logger().info('Iniciando transición LIE → STAND')

    def update(self):
        if not self.active:
            return

        s = self.step / (self.total_steps - 1)
        s = min(max(s, 0.0), 1.0)

        # interpolación suave (0 → 1)
        alpha = 0.5 * (1 - np.cos(np.pi * s))

        z = self.z_lie + alpha * (self.z_stand - self.z_lie)
        self.publish_feet(z)

        self.step += 1

        if self.step >= self.total_steps:
            self.active = False

            done = String()
            done.data = 'LIE_TO_STAND_DONE'
            self.done_pub.publish(done)

            self.get_logger().info('Transición LIE → STAND completada')

    def publish_feet(self, z):
        self.pub_fl.publish(Point(x=self.x, y=self.y_left,  z=z))
        self.pub_fr.publish(Point(x=self.x, y=self.y_right, z=z))
        self.pub_rl.publish(Point(x=self.x, y=self.y_left,  z=z))
        self.pub_rr.publish(Point(x=self.x, y=self.y_right, z=z))


def main():
    rclpy.init()
    node = LieToStandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
