import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np

class FootTrajectoryNode(Node):

    def __init__(self):
        super().__init__('foot_trajectory_node')

        # Publisher
        self.publisher_ = self.create_publisher(Point, 'foot_ref', 10)

        # Timer
        self.dt = 0.01  # 100 Hz
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # Parámetros de la trayectoria (LOS QUE DEFINIMOS)
        self.L = 0.03      # [m]
        self.H = 0.015     # [m]
        self.T = 0.6       # [s]
        self.x0 = 0.0
        # Offset to match leg hip origin used in `quad_ik_node.publish_initial_pose`
        self.y0 = 0.04005
        # Base height (negative -> below hip), trajectory z will be added to this
        self.z0 = -0.10

        self.t_start = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info('Foot trajectory node started')

    def timer_callback(self):
        # Use ROS clock consistently
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.t_start

        # Normalized phase in [0,1)
        tau = (t % self.T) / self.T

        # Interpolador cúbico (velocidad 0 en extremos) s(tau) = 3*tau^2 - 2*tau^3
        s = 3*tau**2 - 2*tau**3

        # Trajectoria tipo arco (parametrizada por s)
        x = self.x0 + (self.L/2)*(1 - np.cos(np.pi*s))
        z = self.z0 + self.H * np.sin(np.pi*s)
        y = self.y0

        # Publicar
        msg = Point()
        msg.x = float(-x)
        msg.y = float(y)
        msg.z = float(z)

        self.publisher_.publish(msg)
        self.get_logger().debug(f'foot_ref published: x={msg.x:.4f} y={msg.y:.4f} z={msg.z:.4f}')

def main(args=None):
    rclpy.init(args=args)
    node = FootTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
