import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np

def inverse_kinematics_leg(x4, y4, z4, L1, L2, L3, leg_side):
    """
    Cinemática inversa para una pata del robot cuadrúpedo.

    Convención:
    - Sentido antihorario positivo
    - Rodilla hacia atrás => q3 < 0
    - Flexión de cadera hacia adelante => q2 > 0
    """

    # ---- Cálculo del término D ----
    D = (y4**2 + z4**2 - L1**2 + x4**2 - L2**2 - L3**2) / (2 * L2 * L3)

    if abs(D) > 1.0:
        raise ValueError("Posición fuera de alcance")

    # ---- Rodilla (rama correcta: q3 negativo al flexionar) ----
    theta3 = np.arctan2(-np.sqrt(1 - D**2), D)

    # ---- Geometría auxiliar ----
    r = np.sqrt(y4**2 + z4**2 - L1**2)

    y_term = x4
    x_term = r

    term2_y = L3 * np.sin(theta3)
    term2_x = L2 + L3 * np.cos(theta3)

    # ---- Ángulo de cadera (q2) ----
    theta2 = -np.arctan2(y_term, x_term) + np.arctan2(term2_y, term2_x)

    # 👉 Corrección de convención (igual que en MATLAB)
    theta2 = -theta2

    # ---- Selección de pata ----
    if leg_side == "left":
        theta1 = -np.arctan2(-z4, y4) - np.arctan2(r, -L1)
        theta1 += np.pi

    elif leg_side == "right":
        theta1 = -np.arctan2(z4, y4) - np.arctan2(r, -L1)

    else:
        raise ValueError("leg_side debe ser 'left' o 'right'")

    return theta1, theta2, theta3

class LegIKNode(Node):
  def __init__(self):
      super().__init__('leg_ik_node')

      self.L1 = 0.04005
      self.L2 = 0.09
      self.L3 = 0.09

      self.leg_side = "left"  # Cambiar a "right" para la pata derecha

      self.sub = self.create_subscription(Point, '/foot_target',self.ik_callback,10)

      self.pub = self.create_publisher(JointState, '/joint_states',10)

      self.get_logger().info("Leg IK Node iniciado")

  def ik_callback(self,msg):
    try:
        q1, q2, q3 = inverse_kinematics_leg(msg.x,msg.y,msg.z,self.L1,self.L2,self.L3,self.leg_side)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        js.name = [
                'front_left_hip_joint',
                'front_left_thigh_joint',
                'front_left_knee_joint'
            ]

        js.position = [q1, q2, q3]

        self.pub.publish(js)
    
    except ValueError as e:
      self.get_logger().warn(str(e))

def main():
    rclpy.init()
    node = LegIKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()