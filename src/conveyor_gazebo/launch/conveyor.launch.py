"""Launch Gazebo Sim with the conveyor world + camera bridge (Phase 1 & 3)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_conveyor_gazebo = get_package_share_directory('conveyor_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            pkg_conveyor_gazebo, 'worlds', 'conveyor.world'),
        description='Absolute path to the Gazebo world file.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Open RViz with the camera image display.',
    )

    # -r : run immediately (unpaused); -v 3 : info-level logging.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [world, ' -r -v 3'],
        }.items(),
    )

    # Bridge the Gazebo camera topics into ROS 2.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(
                pkg_conveyor_gazebo, 'config', 'bridge.yaml'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(pkg_conveyor_gazebo, 'config', 'conveyor.rviz')],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        world_arg,
        rviz_arg,
        gz_sim,
        bridge,
        rviz_node,
    ])
