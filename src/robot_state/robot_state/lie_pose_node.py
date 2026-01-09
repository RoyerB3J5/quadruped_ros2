import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String


class LiePoseNode(Node):
    def __init__(self):
        super().__init__('lie_pose_node')

        self.pub_fl = self.create_publisher(Point, '/foot_ref_lie/front_left', 10)
        self.pub_fr = self.create_publisher(Point, '/foot_ref_lie/front_right', 10)
        self.pub_rl = self.create_publisher(Point, '/foot_ref_lie/rear_left', 10)
        self.pub_rr = self.create_publisher(Point, '/foot_ref_lie/rear_right', 10)

        self.create_subscription(
            String,
            '/robot_state',
            self.state_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.publish_pose)
        self.active = False
        self.last_z = None
        self.z_lie = -0.04

        self.get_logger().info('LiePoseNode listo')

    def state_callback(self, msg):
        is_lie = (msg.data == 'LIE')
        if is_lie and not self.active:
            self.last_z = None 
        self.active = is_lie

    def publish_pose(self):
        if not self.active:
            return

        z = self.z_lie

        if self.last_z == z:
            return
        
        self.pub_fl.publish(Point(x=0.0, y= 0.04005, z=z))
        self.pub_fr.publish(Point(x=0.0, y=-0.04005, z=z))
        self.pub_rl.publish(Point(x=0.0, y= 0.04005, z=z))
        self.pub_rr.publish(Point(x=0.0, y=-0.04005, z=z))
        self.last_z = z


def main():
    rclpy.init()
    node = LiePoseNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
