#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os


class URDFPublisher(Node):
    def __init__(self, urdf_path: str):
        super().__init__('urdf_publisher')
        self.pub = self.create_publisher(String, '/robot_description', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.urdf = ''
        if os.path.exists(urdf_path):
            with open(urdf_path, 'r') as f:
                urdf_text = f.read()
                # Replace model-specific package URIs so RViz can resolve meshes
                urdf_text = urdf_text.replace(
                    'package://urd_robot_cuadrupedo_v5/meshes/',
                    'package://gazebo_simulation/models/urd_robot_cuadrupedo_v5/meshes/'
                )
                self.urdf = urdf_text
        else:
            self.get_logger().error(f'URDF file not found: {urdf_path}')

    def timer_callback(self):
        if not self.urdf:
            return
        msg = String()
        msg.data = self.urdf
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    urdf_path = None
    # accept urdf path as first argument
    if len(sys.argv) > 1:
        urdf_path = sys.argv[1]
    if not urdf_path:
        print('Usage: publish_robot_description.py /path/to/urdf')
        return
    node = URDFPublisher(urdf_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
