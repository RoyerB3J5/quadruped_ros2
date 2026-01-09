import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String


class FootRefMuxNode(Node):
    def __init__(self):
        super().__init__('foot_ref_mux')

        self.state = 'LIE'

        self.create_subscription(String, '/robot_state', self.state_cb, 10)

        # ---- Subs de refs
        self.subs = {}
        self.refs = {}

        sources = ['lie', 'stand', 'transition', 'gait']
        legs = ['front_left', 'front_right', 'rear_left', 'rear_right']

        for src in sources:
            self.refs[src] = {}
            for leg in legs:
                topic = f'/foot_ref_{src}/{leg}'
                self.refs[src][leg] = Point()
                self.create_subscription(
                    Point, topic,
                    lambda msg, s=src, l=leg: self.ref_cb(msg, s, l),
                    10
                )

        self.pub = {
            'front_left':  self.create_publisher(Point, '/foot_ref/front_left', 10),
            'front_right': self.create_publisher(Point, '/foot_ref/front_right', 10),
            'rear_left':   self.create_publisher(Point, '/foot_ref/rear_left', 10),
            'rear_right':  self.create_publisher(Point, '/foot_ref/rear_right', 10),
        }

        self.timer = self.create_timer(0.02, self.publish)

        self.get_logger().info('FootRefMuxNode iniciado')


    def state_cb(self, msg):
        self.state = msg.data

    def ref_cb(self, msg, src, leg):
        self.refs[src][leg] = msg


    def select_source(self):
        if self.state == 'LIE':
            return 'lie'
        elif self.state == 'STAND':
            return 'stand'
        elif self.state.startswith('TRANSITION'):
            return 'transition'
        elif self.state == 'WALK':
            return 'gait'
        else:
            return None

    def publish(self):
        src = self.select_source()
        if src is None:
            return

        for leg, pub in self.pub.items():
            pub.publish(self.refs[src][leg])


def main():
    rclpy.init()
    node = FootRefMuxNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
