from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='robot_state',
      executable='state_manager_node',
      name='state_manager',
      output='screen',
    ),
    Node(
      package='robot_state',
      executable='stand_to_lie_node',
      name='stand_to_lie',
      output='screen',
    ),
    Node(
      package='robot_state',
      executable='lie_to_stand_node',
      name='lie_to_stand',
      output='screen',
    ),
    Node(
      package='robot_state',
      executable='stand_pose_node',
      name='stand_pose',
      output='screen',
    ),
    Node(
      package='robot_state',
      executable='lie_pose_node',
      name='lie_pose',
      output='screen',
    ),
    Node(
      package='robot_state',
      executable='foot_ref_mux_node',
      name='foot_ref_mux',
      output='screen',
    ),
    Node(
      package='robot_kinematics',
      executable='quad_ik_node_2',
      name='quad_ik_2',
      output='screen',
    ),
    Node(
      package='robot_kinematics',
      executable='gait_generator_node',
      name='gait_generator',
      output='screen',
    ),
  ])