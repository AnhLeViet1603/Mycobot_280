"""Top-level launch for the full conveyor sorting pipeline (Phase 8).

Brings up the whole demo with a single command:

    ros2 launch bringup system.launch.py            # headless-ish (Gazebo GUI)
    ros2 launch bringup system.launch.py rviz:=true # + RViz camera view

Pipeline wired end-to-end:

    Gazebo conveyor world + camera bridge   (conveyor_gazebo/conveyor.launch.py)
      -> object_spawner   : drops random-colour cubes at the belt head
      -> vision_node      : HSV colour detection -> /detected_object
      -> decision_node    : blue=KEEP, else -> /reject_object
      -> pusher hardware  : ros2_control pusher   (sorting_robot_description/pusher.launch.py)
      -> pusher_controller_node : /reject_object -> push sequence

Startup is staggered: the world comes up first, then the pusher is spawned and
its controllers loaded, and only then does the spawner begin releasing cubes,
so the pusher is in place before the first cube arrives. The vision, decision
and controller nodes start early and simply idle until data flows.

All node parameters live in ``config/system_params.yaml`` so timing and the
accepted colour can be tuned in one place.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('bringup')
    pkg_conveyor = get_package_share_directory('conveyor_gazebo')
    pkg_desc = get_package_share_directory('sorting_robot_description')

    params = os.path.join(pkg_bringup, 'config', 'system_params.yaml')

    rviz = LaunchConfiguration('rviz')

    # 1) Gazebo world + camera bridge (+ optional RViz).
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_conveyor, 'launch', 'conveyor.launch.py')),
        launch_arguments={'rviz': rviz}.items(),
    )

    # 2) Perception + decision. These just wait for topics, so start early.
    vision = Node(
        package='vision_node', executable='vision_node',
        name='vision_node', output='screen', parameters=[params],
    )
    decision = Node(
        package='decision_node', executable='decision_node',
        name='decision_node', output='screen', parameters=[params],
    )
    pusher_controller = Node(
        package='sorting_robot_controller', executable='pusher_controller_node',
        name='pusher_controller_node', output='screen', parameters=[params],
    )

    # 3) Pusher hardware: robot_state_publisher + spawn + ros2_control.
    #    start_world:=false — the world above is already coming up.
    pusher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'pusher.launch.py')),
        launch_arguments={'start_world': 'false'}.items(),
    )

    # 4) Cube spawner. Last, so the pusher is spawned and belt running first.
    spawner = Node(
        package='object_spawner', executable='spawner_node',
        name='object_spawner', output='screen', parameters=[params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz with the camera image display.'),
        world,
        # Perception/decision/controller can come up almost immediately.
        TimerAction(period=3.0, actions=[vision, decision, pusher_controller]),
        # Give Gazebo a few seconds before spawning the pusher into it.
        TimerAction(period=6.0, actions=[pusher]),
        # Start releasing cubes once the pusher has had time to load.
        TimerAction(period=15.0, actions=[spawner]),
    ])
