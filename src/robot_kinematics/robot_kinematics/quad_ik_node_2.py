import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from robot_kinematics.inverse_kinematics import inverse_kinematics_leg

class QuadIKNode(Node):

    def __init__(self):
        super().__init__('quad_ik_node')

        # Longitudes de las piernas
        self.L1 = 0.04005
        self.L2 = 0.09
        self.L3 = 0.09

        # Orden de joints según URDF / controller
        self.joint_names = [
            'Motor1', 'Motor5', 'Motor9',
            'Motor2', 'Motor6', 'Motor10',
            'Motor3', 'Motor7', 'Motor11',
            'Motor4', 'Motor8', 'Motor12',
        ]

        self.joint_positions = [0.0] * 12

        self.legs = {
            'front_left' : {'side':'left',  'idx' : [6,7,8]},
            'front_right': {'side':'right', 'idx' : [0,1,2]},
            'rear_left'  : {'side':'left',  'idx' : [9,10,11]},
            'rear_right' : {'side':'right', 'idx' : [3,4,5]},
        }

        self.foot_refs = {leg: None for leg in self.legs.keys()}

        # Suscripciones a referencias de pies
        for leg in self.legs.keys():
            self.create_subscription(
                Point, f'/foot_ref/{leg}',
                lambda msg, leg=leg: self.foot_callback(msg, leg),
                10
            )

        # Publicador ForwardCommandController
        self.pub = self.create_publisher(Float64MultiArray, '/leg_controller/commands', 10)

        # Timer de actualización
        self.timer = self.create_timer(0.01, self.update_ik)

        self.get_logger().info('Quad IK node (Forward) started')

    def foot_callback(self, msg, leg):
        self.foot_refs[leg] = msg

    def update_ik(self):
        # Verifica que todas las patas tengan referencia
        if any(msg is None for msg in self.foot_refs.values()):
            return

        for leg, msg in self.foot_refs.items():
            cfg = self.legs[leg]
            try:
                q1, q2, q3 = inverse_kinematics_leg(
                    msg.x, msg.y, msg.z,
                    self.L1, self.L2, self.L3, cfg['side']
                )
            except ValueError:
                continue
            
            i0, i1, i2 = cfg['idx']
            self.joint_positions[i0] = q1
            self.joint_positions[i1] = q2
            self.joint_positions[i2] = q3

        self.publish_forward_command()

    def publish_forward_command(self):
        msg = Float64MultiArray()
        msg.data = self.joint_positions  # en radianes
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = QuadIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
