"""Xem myCobot 280 trong RViz (không cần Gazebo).

Chạy robot_state_publisher + joint_state_publisher_gui để kéo thanh trượt kiểm tra
các khớp (kể cả mimic của gripper). Dùng use_gz:=false để chỉ nạp hình học.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("mycobot_description")
    xacro_file = os.path.join(pkg, "urdf", "mycobot_280.urdf.xacro")
    rviz_config = os.path.join(pkg, "rviz", "view_robot.rviz")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ", xacro_file,
                " use_gz:=false",
                " use_world_fixed:=true",
            ]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=None,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
