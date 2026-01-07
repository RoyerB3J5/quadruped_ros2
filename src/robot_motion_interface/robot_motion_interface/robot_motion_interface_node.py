import rclpy
from rclpy.node import Node
from robot_msgs.msg import MotionCommand

class RobotMotionInterfaceNode(Node):
    def __init__(self):
        super().__init__('robot_motion_interface')

        self.pub = self.create_publisher(MotionCommand, '/motion_command', 10)

        # Estado interno (teach pendant)
        self.command = "stand"
        self.gait = "trot"
        self.velocity = 0.15
        self.step_height = 0.01
        self.step_length = 0.035

        self.get_logger().info(
            "\nTeach Pendant Controls:\n"
            " w : walk\n"
            " s : stand\n"
            " l : lie\n"
            " t : trot gait\n"
            " c : crawl gait\n"
            " + : increase velocity\n"
            " - : decrease velocity\n"
            " h : increase step height\n"
            " n : decrease step height\n"
            " q : quit\n"
        )

        self.run_keyboard()

    def publish(self):
        msg = MotionCommand()
        msg.command = self.command
        msg.gait = self.gait
        msg.velocity = self.velocity
        msg.step_height = self.step_height
        msg.step_length = self.step_length

        self.pub.publish(msg)

        self.get_logger().info(
            f"CMD={msg.command} | v={msg.velocity:.2f} "
            f"h={msg.step_height:.3f} L={msg.step_length:.3f}"
        )

    def run_keyboard(self):
        while rclpy.ok():
            key = input(">> ").lower()

            if key == 'w':
                self.command = "walk"

            elif key == 's':
                self.command = "stand"

            elif key == 'l':
                self.command = "lie"

            elif key == 't':
                self.gait = "trot"

            elif key == 'c':
                self.gait = "crawl"

            elif key == '+':
                self.velocity = min(self.velocity + 0.02, 0.4)

            elif key == '-':
                self.velocity = max(self.velocity - 0.02, 0.0)

            elif key == 'h':
                self.step_height = min(self.step_height + 0.005, 0.05)

            elif key == 'n':
                self.step_height = max(self.step_height - 0.005, 0.0)

            elif key == 'q':
                break

            else:
                print("Unknown key")
                continue

            self.publish()

def main(args=None):
    rclpy.init(args=args)
    node = RobotMotionInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
