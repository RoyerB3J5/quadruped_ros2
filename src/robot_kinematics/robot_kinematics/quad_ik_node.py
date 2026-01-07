import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robot_kinematics.inverse_kinematics import inverse_kinematics_leg


class QuadIKNode(Node):

    def __init__(self):
        super().__init__('quad_ik_node')

        # Longitudes
        self.L1 = 0.04005
        self.L2 = 0.09
        self.L3 = 0.09

        # Orden COMPLETO de joints (DEBE coincidir con el URDF)
        # MUST match controller ordering used by Gazebo (see joint_names_*.yaml)
        # Ordering: front_right(Motor1,5,9), rear_right(Motor2,6,10),
        #           front_left(Motor3,7,11), rear_left(Motor4,8,12)
        self.joint_names = [
            'Motor1', 'Motor5', 'Motor9',
            'Motor2', 'Motor6', 'Motor10',
            'Motor3', 'Motor7', 'Motor11',
            'Motor4', 'Motor8', 'Motor12',
        ]

        # Estado interno de articulaciones
        self.joint_positions = [0.0] * 12

        # Índices de las patas dentro del vector `joint_positions`
        # Después de reordenar `joint_names` según el controlador:
        # front_right -> indices 0,1,2 (Motor1,5,9)
        # rear_right  -> indices 3,4,5 (Motor2,6,10)
        # front_left  -> indices 6,7,8 (Motor3,7,11)
        # rear_left   -> indices 9,10,11 (Motor4,8,12)
        self.legs = {
            'front_left' : {'side':'left',  'idx' : [6,7,8]},
            'front_right': {'side':'right', 'idx' : [0,1,2]},
            'rear_left'  : {'side':'left',  'idx' : [9,10,11]},
            'rear_right' : {'side':'right', 'idx' : [3,4,5]},
        }

        self.foot_refs = {
            'front_left': None,
            'front_right': None,
            'rear_left': None,
            'rear_right': None,
        }

        # Suscribirse a las patas 
        for leg in self.legs.keys():
            self.create_subscription(
                Point, f'/foot_ref/{leg}',
                lambda msg, leg=leg: self.foot_callback(msg,leg),
                10
            )
        
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self.timer = self.create_timer(0.01,self.update_ik)

        self.get_logger().info('Quad IK node started')

    # ---------------------------------------------------------
    # ---------------------------------------------------------

    def foot_callback(self, msg, leg):
        self.foot_refs[leg] = msg
        self.get_logger().debug(f'Received foot_ref for {leg}: x={msg.x:.4f} y={msg.y:.4f} z={msg.z:.4f}')
    # ---------------------------------------------------------
    def update_ik(self):
        if any(msg is None for msg in self.foot_refs.values()):
            return
        
        for leg, msg in self.foot_refs.items():
            if msg is None:
                continue 

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

        self.publish_joint_states()

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.joint_positions

        self.pub.publish(js)
def main():
    rclpy.init()
    node = QuadIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
