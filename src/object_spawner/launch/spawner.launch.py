"""Launch the object spawner node (Phase 2)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    spawn_period = LaunchConfiguration('spawn_period')
    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='conveyor_world'),
        DeclareLaunchArgument('spawn_period', default_value='3.0'),
        Node(
            package='object_spawner',
            executable='spawner_node',
            name='object_spawner',
            output='screen',
            parameters=[{
                'world': world,
                'spawn_period': spawn_period,
            }],
        ),
    ])
