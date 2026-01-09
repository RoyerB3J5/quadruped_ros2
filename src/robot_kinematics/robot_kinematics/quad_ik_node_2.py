import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String  # Usamos String para enviar JSON
from robot_kinematics.inverse_kinematics import inverse_kinematics_leg
import json

class QuadIKNode(Node):

    def __init__(self):
        super().__init__('quad_ik_node')

        # Longitudes de las piernas
        self.L1 = 0.04005
        self.L2 = 0.09
        self.L3 = 0.09

        # Definición de patas
        self.legs = {
            'front_left' : {'side':'left'},
            'front_right': {'side':'right'},
            'rear_left'  : {'side':'left'},
            'rear_right' : {'side':'right'},
        }
        
        self.foot_refs = {leg: None for leg in self.legs.keys()}

        for leg in self.legs.keys():
            self.create_subscription(
                Point, f'/foot_ref/{leg}',
                lambda msg, leg=leg: self.foot_callback(msg, leg),
                10
            )

        self.pub_angles = self.create_publisher(String, '/leg_angles', 10)

        self.timer = self.create_timer(0.01, self.update_ik)

        self.get_logger().info('Quad IK node (Coppelia) started')

    def foot_callback(self, msg, leg):
        self.foot_refs[leg] = msg

    def update_ik(self):
        if any(msg is None for msg in self.foot_refs.values()):
            return

        angles_dict = {}
        for leg, msg in self.foot_refs.items():
            try:
                q1, q2, q3 = inverse_kinematics_leg(
                    msg.x, msg.y, msg.z,
                    self.L1, self.L2, self.L3,
                    self.legs[leg]['side']
                )
            except ValueError:
                q1 = q2 = q3 = 0.0
            angles_dict[leg] = [q1, q2, q3]

        angles_list = []
        angles_list.extend(angles_dict['front_left'])
        angles_list.extend(angles_dict['front_right'])
        angles_list.extend(angles_dict['rear_left'])
        angles_list.extend(angles_dict['rear_right'])

        msg_str = String()
        msg_str.data = json.dumps(angles_list)
        self.pub_angles.publish(msg_str)

def main():
    rclpy.init()
    node = QuadIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
