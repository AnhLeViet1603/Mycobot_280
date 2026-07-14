"""Spawn the servo pusher into the conveyor world (Phase 6).

Brings up (optionally) the Gazebo conveyor world, publishes the pusher
description, spawns it into Gazebo, and starts the controllers so the
prismatic joint can be commanded by hand:

    ros2 topic pub -1 /pusher_position_controller/commands \
        std_msgs/Float64MultiArray "{data: [-0.8]}"   # extend
    ros2 topic pub -1 /pusher_position_controller/commands \
        std_msgs/Float64MultiArray "{data: [0.0]}"    # retract
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_desc = get_package_share_directory('sorting_robot_description')
    pkg_conveyor = get_package_share_directory('conveyor_gazebo')

    controllers_config = os.path.join(
        pkg_desc, 'config', 'pusher_controllers.yaml')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'pusher.urdf.xacro')

    # Process xacro up front, injecting the controllers YAML path.
    robot_description = xacro.process_file(
        xacro_file,
        mappings={'controllers_config': controllers_config},
    ).toxml()

    start_world = LaunchConfiguration('start_world')

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_conveyor, 'launch', 'conveyor.launch.py')),
        condition=IfCondition(start_world),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    spawn_pusher = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'pusher'],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    pusher_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pusher_position_controller'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_world', default_value='true',
            description='Also launch the Gazebo conveyor world.'),
        world_launch,
        robot_state_publisher,
        spawn_pusher,
        joint_state_broadcaster,
        pusher_controller,
    ])
