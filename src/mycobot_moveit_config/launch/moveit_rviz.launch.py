"""RViz với MotionPlanning plugin để test plan/execute bằng chuột.

Chạy:  ros2 launch mycobot_moveit_config moveit_rviz.launch.py
(cần move_group.launch.py + bringup đang chạy).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name="mycobot_moveit_config")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    rviz_config = os.path.join(
        get_package_share_directory("mycobot_moveit_config"), "rviz", "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([rviz_node])
