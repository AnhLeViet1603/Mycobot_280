"""Phase 7: pusher hardware + closed-loop reject controller.

Reuses ``sorting_robot_description/pusher.launch.py`` to bring up the Gazebo
world, spawn the pusher, and start ros2_control (joint_state_broadcaster +
pusher_position_controller), then adds ``pusher_controller_node`` which turns
``/reject_object`` commands into a push sequence.

    ros2 launch sorting_robot_controller pusher_control.launch.py

Set ``start_world:=false`` to attach to an already-running world.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_desc = get_package_share_directory('sorting_robot_description')

    start_world = LaunchConfiguration('start_world')

    pusher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'pusher.launch.py')),
        launch_arguments={'start_world': start_world}.items(),
    )

    pusher_controller_node = Node(
        package='sorting_robot_controller',
        executable='pusher_controller_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_world', default_value='true',
            description='Also launch the Gazebo conveyor world.'),
        pusher_launch,
        pusher_controller_node,
    ])
