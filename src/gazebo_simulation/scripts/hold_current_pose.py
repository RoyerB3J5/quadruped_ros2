#!/usr/bin/env python3
"""Publish current /joint_states as a one-point JointTrajectory repeatedly.

Purpose: make the controller receive a safe, current-position setpoint
immediately after (or around) activation so the robot doesn't get a
large instantaneous command that throws it off balance.

Usage:
  python3 src/gazebo_simulation/scripts/hold_current_pose.py

It publishes to the topic `leg_controller/joint_trajectory` 10-15 times
over ~0.75s and then exits.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class HoldCurrentPose(Node):
    # Joint ordering expected by leg_controller (must match joint_controllers.yaml)
    CONTROLLER_JOINT_ORDER = [
        'Motor1','Motor5','Motor9',
        'Motor2','Motor6','Motor10',
        'Motor3','Motor7','Motor11',
        'Motor4','Motor8','Motor12'
    ]

    def __init__(self):
        super().__init__('hold_current_pose')
        self.pub = self.create_publisher(JointTrajectory, 'leg_controller/joint_trajectory', 10)
        self.sub = self.create_subscription(JointState, 'joint_states', self.joint_state_cb, 10)
        self.published = False

    def joint_state_cb(self, msg: JointState):
        if self.published:
            return
        if not msg.position or not msg.name:
            self.get_logger().warning('Received joint_states without positions yet')
            return

        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}

        traj = JointTrajectory()
        traj.joint_names = list(self.CONTROLLER_JOINT_ORDER)

        point = JointTrajectoryPoint()
        positions = []
        missing = []
        for j in traj.joint_names:
            if j in name_to_pos:
                positions.append(name_to_pos[j])
            else:
                positions.append(0.0)
                missing.append(j)

        if missing:
            self.get_logger().warning(f'Missing joints in /joint_states: {missing}')

        point.positions = positions
        # Slightly longer time to soften controller response
        point.time_from_start = Duration(sec=1, nanosec=500_000_000)
        traj.points = [point]

        self.get_logger().info(f'Publishing hold trajectory for {len(traj.joint_names)} joints (ordered)')

        # Publish several times to increase chance the controller receives it
        for _ in range(20):
            self.pub.publish(traj)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.published = True
        self.get_logger().info('Hold trajectory published — exiting')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = HoldCurrentPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
