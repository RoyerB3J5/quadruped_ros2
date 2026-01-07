from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('gazebo_simulation')

    urdf_file = os.path.join(
        pkg_path,
        'models',
        'urd_robot_cuadrupedo_v5',
        'urd_robot_cuadrupedo_v5.urdf'
    )

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['cat ', urdf_file]), value_type=str)
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                pkg_path,
                'rviz',
                'urd_v5.rviz'
            )]
        )
    ])
