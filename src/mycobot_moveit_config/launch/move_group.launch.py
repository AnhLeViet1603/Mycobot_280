"""Khởi động move_group cho myCobot 280.

Dùng chung robot_description với bringup (mount_z=0.42), nhưng use_gz:=false
(không cần khối ros2_control/plugin gz cho planning). move_group nhận /joint_states
từ Gazebo qua bridge, và execute trajectory tới arm_controller/gripper_controller (JTC).

Chạy:  ros2 launch mycobot_moveit_config move_group.launch.py
(bringup.launch.py phải chạy trước để có controllers + /joint_states).
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name="mycobot_moveit_config")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            # Cho phép execute trajectory MoveIt sinh ra qua simple controller manager.
            {"publish_robot_description_semantic": True},
        ],
    )

    return LaunchDescription([move_group_node])
