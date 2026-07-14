"""Launch the decision node (Phase 5)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    accepted_color = LaunchConfiguration('accepted_color')

    return LaunchDescription([
        DeclareLaunchArgument('accepted_color', default_value='blue'),
        Node(
            package='decision_node',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[{
                'accepted_color': accepted_color,
            }],
        ),
    ])
