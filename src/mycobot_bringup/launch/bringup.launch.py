"""Bringup Gazebo cho myCobot 280.

Khởi động: Gazebo Sim (pick_world) -> robot_state_publisher -> spawn robot lên bàn
-> bridge /clock -> spawn controllers (stagger để tránh lỗi load controller).

Chạy:  ros2 launch mycobot_bringup bringup.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("mycobot_bringup")
    desc_share = get_package_share_directory("mycobot_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(desc_share, "urdf", "mycobot_280.urdf.xacro")
    controllers_yaml = os.path.join(bringup_share, "config", "controllers.yaml")
    world_file = os.path.join(bringup_share, "worlds", "pick_world.sdf")
    bridge_config = os.path.join(bringup_share, "config", "ros_gz_bridge.yaml")

    # Cho Gazebo tìm được mesh package://mycobot_description/...
    set_gz_resource = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.dirname(desc_share)
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ", xacro_file,
                " use_gz:=true",
                " use_world_fixed:=true",
                " mount_z:=0.42",
                " controllers_config:=", controllers_yaml,
            ]),
            value_type=str,
        )
    }

    # Gazebo Sim với world (-r: chạy ngay). gui:=false -> headless (-s) để test nhanh.
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-s -r {world_file}"}.items(),
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Spawn robot từ topic robot_description, đặt lên mặt bàn (z=0.4)
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "mycobot_280",
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["--ros-args", "-p", f"config_file:={bridge_config}"],
    )

    # Spawner controllers — stagger để controller_manager sẵn sàng và không đua nhau
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="true: Gazebo có GUI; false: headless (-s) để test nhanh.",
        ),
        set_gz_resource,
        gz_sim_gui,
        gz_sim_headless,
        robot_state_publisher,
        clock_bridge,
        # spawn robot sau khi RSP publish robot_description
        TimerAction(period=3.0, actions=[spawn_robot]),
        # nạp controllers tuần tự: jsb -> arm -> gripper
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_robot, on_exit=[jsb_spawner])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=arm_spawner, on_exit=[gripper_spawner])
        ),
    ])
