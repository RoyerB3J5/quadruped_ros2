from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      output='screen',
      parameters=[{
        'robot_description': open(
          '/home/royerbj/robot_ws/src/robot_description/urdf/hyperdog.urdf'
        ).read()
      }]
    ),
    Node(
      package='rviz2',
      executable='rviz2',
      output='screen',
    )
  ])

"""Node(
      package='joint_state_publisher_gui',
      executable='joint_state_publisher_gui',
      output='screen'
),"""