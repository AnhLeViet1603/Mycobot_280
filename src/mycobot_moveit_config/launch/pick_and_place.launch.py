"""Chạy node pick_and_place (moveit_py) với đầy đủ param MoveIt (Phase 7).

moveit_py cần các param robot_description / semantic / kinematics / planning pipelines /
moveit_controllers nạp vào chính node -> lấy từ MoveItConfigsBuilder (như move_group).

Yêu cầu đang chạy: bringup (Gazebo + controllers) + spawn (vật) + grasp (grasp_manager).
  ros2 launch mycobot_moveit_config pick_and_place.launch.py
  ros2 launch mycobot_moveit_config pick_and_place.launch.py num_objects:=3
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name="mycobot_moveit_config")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    num_objects = LaunchConfiguration("num_objects")

    pnp_node = Node(
        package="mycobot_demo",
        executable="pick_and_place",
        name="pick_and_place",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"num_objects": ParameterValue(num_objects, value_type=int)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("num_objects", default_value="5"),
        pnp_node,
    ])
