from setuptools import find_packages, setup

package_name = 'robot_kinematics'

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
          #'leg_kinematics_node = robot_kinematics.leg_kinematics_node:main',
          'leg_ik_node = robot_kinematics.leg_ik_node:main',
          'quad_ik_node = robot_kinematics.quad_ik_node:main',
          'foot_trajectory_node = robot_kinematics.foot_trajectory_node:main',
          'trajectory_foot_node = robot_kinematics.trajectory_foot_node:main',
          'gait_generator_node = robot_kinematics.gait_generator_node:main',
          'foot_trajectory_viz = robot_kinematics.foot_trajectory_viz:main',
          'quad_ik_node_2 = robot_kinematics.quad_ik_node_2:main',
        ],
    },
)
