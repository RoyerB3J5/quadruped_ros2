from setuptools import find_packages, setup

package_name = 'robot_state'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='royerbj',
    maintainer_email='royerbj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                    'stand_pose_node = robot_state.stand_pose_node:main',
                    'lie_pose_node   = robot_state.lie_pose_node:main',
                    'lie_to_stand_node = robot_state.lie_to_stand_node:main',
                    'state_manager_node = robot_state.state_manager_node:main',
                    'stand_to_lie_node = robot_state.stand_to_lie_node:main',
                    'foot_ref_mux_node = robot_state.foot_ref_mux_node:main',
        ],
    },
)
