from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    pkg_path = FindPackageShare("gazebo_simulation").find("gazebo_simulation")

    world_path = os.path.join(pkg_path, "worlds", "empty.world")

    urdf_path = os.path.join(
        pkg_path,
        "models",
        "urd_robot_cuadrupedo_v5",
        "urd_robot_cuadrupedo_v5.urdf"
    )

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("gazebo_ros").find("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={"world": world_path}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen"
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "quadruped",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.30"
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn
    ])
