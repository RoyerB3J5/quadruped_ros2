import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
from robot_msgs.msg import MotionCommand
from rclpy.qos import QoSProfile, DurabilityPolicy


class RobotState(Enum):
    LIE = 0
    TRANSITION_LIE_TO_STAND = 1
    STAND = 2
    WALK = 3
    TRANSITION_WALK_TO_STAND = 4
    TRANSITION_STAND_TO_LIE = 5


class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager')

        self.state = RobotState.LIE

        self.cmd_sub = self.create_subscription(
            MotionCommand,
            '/motion_command',
            self.command_callback,
            10
        )

        self.done_sub = self.create_subscription(
            String,
            '/transition_done',
            self.done_callback,
            10
        )

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            String,
            '/robot_state',
            qos_profile=qos
        )

        self.publish_state()
        self.get_logger().info('State Manager iniciado')

    # ---------------------------------------------

    def set_state(self, new_state: RobotState):
        if new_state == self.state:
            return
        self.state = new_state
        self.publish_state()
        self.get_logger().info(f'Estado -> {self.state.name}')

    # ---------------------------------------------

    def command_callback(self, msg):
        command = msg.command.lower()

        # Bloquear comandos durante transición
        if self.state in (
            RobotState.TRANSITION_LIE_TO_STAND,
            RobotState.TRANSITION_STAND_TO_LIE,
            RobotState.TRANSITION_WALK_TO_STAND
        ):
            self.get_logger().warn('Robot en transición, comando ignorado')
            return

        if command == "walk" and self.state == RobotState.STAND:
            self.set_state(RobotState.WALK)

        elif command == "stand" and self.state == RobotState.WALK:
            self.set_state(RobotState.TRANSITION_WALK_TO_STAND)

        elif command == "stand" and self.state == RobotState.LIE:
            self.set_state(RobotState.TRANSITION_LIE_TO_STAND)

        elif command == "lie" and self.state == RobotState.STAND:
            self.set_state(RobotState.TRANSITION_STAND_TO_LIE)

    # ---------------------------------------------

    def done_callback(self, msg):

        if msg.data == 'LIE_TO_STAND_DONE' and self.state == RobotState.TRANSITION_LIE_TO_STAND:
            self.set_state(RobotState.STAND)

        elif msg.data == 'STAND_TO_LIE_DONE' and self.state == RobotState.TRANSITION_STAND_TO_LIE:
            self.set_state(RobotState.LIE)

        elif msg.data == 'WALK_TO_STAND_DONE' and self.state == RobotState.TRANSITION_WALK_TO_STAND:
            self.set_state(RobotState.STAND)

    # ---------------------------------------------

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = StateManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
