from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('gazebo_simulation')

    # ---------------- URDF ----------------
    urdf_path = os.path.join(
        pkg_share,
        'models',
        'urd_robot_cuadrupedo_v5',
        'urd_robot_cuadrupedo_v5.urdf'
    )

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # ---------------- CONTROLLERS YAML ----------------
    controllers_yaml = os.path.join(
        pkg_share,
        'models',
        'urd_robot_cuadrupedo_v5',
        'config',
        'joint_controllers.yaml'
    )

    # ---------------- GAZEBO ----------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        # Pass our custom world to Gazebo to improve contact stability
        launch_arguments={'verbose': 'true', 'world': os.path.join(pkg_share, 'worlds', 'urd_robot_stable.world')}.items(),
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'models')
    )

    # ---------------- ROBOT STATE PUBLISHER ----------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        output='screen',
    )

    # ---------------- SPAWN ROBOT ----------------
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'urd_robot_v5',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.043' 
        ],
        output='screen',
    )

    # ---------------- CONTROLLERS ----------------
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            controllers_yaml
        ],
        output='screen'
    )

    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'leg_controller',
            '--param-file',
            controllers_yaml
        ],
        output='screen'
    )

    # ---------------- LAUNCH ----------------
    # Use event handlers to start controller spawners immediately after spawn_entity finishes,
    # avoiding races with Gazebo/controller_manager initialization.
    start_joint_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_spawner],
        )
    )

    start_leg_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[leg_controller_spawner],
        )
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        start_joint_handler,
        start_leg_handler,
    ])
