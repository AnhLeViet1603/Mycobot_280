"""Chạy node pick_and_place (moveit_py) với đầy đủ param MoveIt (Phase 7).

moveit_py cần các param robot_description / semantic / kinematics / planning pipelines /
moveit_controllers nạp vào chính node MoveItPy tạo ra (tên "pick_and_place_moveit").

Vì tiến trình tạo NHIỀU node (MoveItPy + node publisher attach/detach) mà `-r __node:=` sẽ
ép TẤT CẢ node về cùng 1 tên -> KHÔNG đặt name cho launch Node. Thay vào đó ghi param dưới
wildcard `/**` để mọi node trong tiến trình đều thấy (kể cả node nội bộ của MoveItPy).

Yêu cầu đang chạy: bringup (Gazebo + controllers) + spawn (vật) + grasp (grasp_manager).
  ros2 launch mycobot_moveit_config pick_and_place.launch.py
  ros2 launch mycobot_moveit_config pick_and_place.launch.py num_objects:=3
"""
import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _launch_setup(context):
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name="mycobot_moveit_config")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    num_objects = int(LaunchConfiguration("num_objects").perform(context))

    params = moveit_config.to_dict()
    params["use_sim_time"] = True
    params["num_objects"] = num_objects

    # moveit_py (moveit_cpp) đọc pipeline theo cấu trúc KHÁC move_group:
    #   planning_pipelines.pipeline_names  (dict) thay vì planning_pipelines (list).
    # Config từng pipeline (key "ompl") vẫn ở top-level. Thêm plan_request_params làm mặc định
    # cho PlanningComponent.plan().
    params["planning_pipelines"] = {"pipeline_names": ["ompl"]}
    params["plan_request_params"] = {
        "planning_attempts": 5,
        "planning_pipeline": "ompl",
        "planner_id": "RRTConnectkConfigDefault",
        "max_velocity_scaling_factor": 0.3,
        "max_acceleration_scaling_factor": 0.3,
        "planning_time": 5.0,
    }

    # Ghi ra file YAML với wildcard /** để mọi node trong tiến trình đọc được param MoveIt.
    fd, path = tempfile.mkstemp(prefix="pnp_params_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump({"/**": {"ros__parameters": params}}, f)

    pnp_node = Node(
        package="mycobot_demo",
        executable="pick_and_place",
        output="screen",
        parameters=[path],
    )
    return [pnp_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_objects", default_value="5"),
        OpaqueFunction(function=_launch_setup),
    ])
