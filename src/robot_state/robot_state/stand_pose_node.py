import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from robot_msgs.msg import MotionCommand

class StandPoseNode(Node):
    def __init__(self):
        super().__init__('stand_pose_node')

        self.pub_fl = self.create_publisher(Point, '/foot_ref_stand/front_left', 10)
        self.pub_fr = self.create_publisher(Point, '/foot_ref_stand/front_right', 10)
        self.pub_rl = self.create_publisher(Point, '/foot_ref_stand/rear_left', 10)
        self.pub_rr = self.create_publisher(Point, '/foot_ref_stand/rear_right', 10)

        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10
        )

        self.cmd_sub = self.create_subscription(
            MotionCommand,
            '/motion_command',
            self.command_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.active = False

        self.current_z = 0.10
        self.start_z = 0.10
        self.target_z = 0.10
        
        self.step = 0
        self.total_steps = 50
        self.interpolating = False

    def command_callback(self, msg):
        if abs(msg.z_height - self.target_z) > 1e-4:
            self.start_z = self.current_z
            self.target_z = msg.z_height
            self.step = 0
            self.interpolating = True

    def state_callback(self, msg):
        if msg.data == 'STAND':
            self.active = True
        else:
            self.active = False

    def publish_pose(self):
        if not self.active:
            return

        if self.interpolating:
            s = self.step / (self.total_steps - 1)
            s = np.clip(s, 0.0, 1.0)

            alpha = 0.5*(1.0 - np.cos(np.pi*s))
            z = self.start_z + alpha*(self.target_z - self.start_z)

            self.current_z = z
            self.step += 1

            if self.step >= self.total_steps:
                self.current_z = self.target_z
                self.interpolating = False
        else:
            z = self.current_z

        self.pub_fl.publish(Point(x=0.0, y= 0.04005, z=-z))
        self.pub_fr.publish(Point(x=0.0, y=-0.04005, z=-z))
        self.pub_rl.publish(Point(x=0.0, y= 0.04005, z=-z))
        self.pub_rr.publish(Point(x=0.0, y=-0.04005, z=-z))

def main():
    rclpy.init()
    node = StandPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
