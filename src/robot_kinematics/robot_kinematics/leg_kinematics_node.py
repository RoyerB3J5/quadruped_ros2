# Creamos el nodo de ROS2 para la cinemática de las patas del robot
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

from robot_kinematics.leg_solver import solve_leg

class LegKinematicsNode(Node):
  def __init__(self):
    super().__init__('leg_kinematics_node')

    self.publisher_ = self.create_publisher(JointState, '/joint_states',10)

    self.timer = self.create_timer(0.05, self.timer_callback)

    #Geometría de la pata 
    self.geometry = {
      "L1": 0.04005,
      "L2": 0.09,
      "L3": 0.09,
    }

    #Postura inicial
    self.foot_positions = {
      "front_left": (0.0, 0.04005, -0.10),
      "front_right": (0.0, -0.04005, -0.10),
      "rear_left": (0.0, 0.04005, -0.10),
      "rear_right": (0.0, -0.04005, -0.10),  
    }

    self.get_logger().info("Leg Kinematics Node iniciado")
  
  """def timer_callback(self):
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()

    for leg, (x,y,z) in self.foot_positions.items():
      q1, q2, q3 = solve_leg(x=x, y=y, z=z, leg_name=leg, geometry=self.geometry)

      msg.name.extend([
        f"{leg}_q1",
        f"{leg}_q2",
        f"{leg}_q3",
      ])

      msg.position.extend([q1, q2, q3])
    
    self.publisher_.publish(msg)"""
  
  def timer_callback(self):
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()

    for leg in ["front_left", "front_right", "rear_left", "rear_right"]:
      q1 = 0.4
      q2 = -0.7
      q3 = -1.4

      msg.name.extend([
        f"{leg}_hip_joint",
        f"{leg}_thigh_joint",
        f"{leg}_knee_joint",
      ])

      msg.position.extend([q1, q2, q3])

    self.publisher_.publish(msg)

def main(args=None):
  rclpy.init(args=args)
  node = LegKinematicsNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

