import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
import time
import math

class FootTrajectoryNode(Node):

    def __init__(self):
        super().__init__('foot_trajectory_node')

        self.pub = self.create_publisher(Point, '/foot_ref', 10)

        # Parámetros (hardcodeados para pruebas)
        self.step_length = 0.03      # [m]
        self.step_height = 0.015      # [m]
        self.cycle_time = 1.0        # [s]
        self.swing_time = 0.4        # [s]

        self.timer_period = 0.01     # 100 Hz

        # Use high-resolution float seconds (nanoseconds -> seconds)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.phase = "swing"
        self.z0 = -0.10
        # Puntos inicial y final
        self.start_pnt = np.array([self.step_length/2, 0.04005, self.z0])
        self.end_pnt   = np.array([-self.step_length/2, 0.04005, self.z0])

        # Create timer after all attributes are initialized so the callback
        # won't run before `start_time` and points are set.
        self.timer = self.create_timer(self.timer_period, self.update)

        self.get_logger().info('Foot trajectory node started')


    def update(self):
      now = self.get_clock().now().nanoseconds / 1e9
      t = (now - self.start_time) % self.cycle_time

      msg = Point()

      # --- SWING ---
      if t < self.swing_time:
          s = t / self.swing_time
          msg.x = self.start_pnt[0] + (self.end_pnt[0] - self.start_pnt[0]) * s
          msg.y = self.start_pnt[1]
          msg.z = self.z0 + self.step_height * math.sin(math.pi * s)

      # --- STANCE ---
      else:
        s = (t - self.swing_time) / (self.cycle_time - self.swing_time)
        msg.x = self.end_pnt[0] + (self.start_pnt[0] - self.end_pnt[0]) * s
        msg.y = self.start_pnt[1]
        msg.z = self.z0

      self.pub.publish(msg)



def main():
    rclpy.init()
    node = FootTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
