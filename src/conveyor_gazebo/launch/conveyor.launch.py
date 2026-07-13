"""Launch Gazebo Sim with the conveyor world (Phase 1)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_conveyor_gazebo = get_package_share_directory('conveyor_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            pkg_conveyor_gazebo, 'worlds', 'conveyor.world'),
        description='Absolute path to the Gazebo world file.',
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

    return LaunchDescription([
        world_arg,
        gz_sim,
    ])
