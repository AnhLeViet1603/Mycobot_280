"""Demo 1 lệnh: Gazebo + robot + controllers -> spawn vật -> grasp_manager -> pick-and-place.

  ros2 launch mycobot_bringup demo.launch.py
  ros2 launch mycobot_bringup demo.launch.py gui:=false auto_pick:=false num_objects:=3

Gộp bằng cách include lại các launch của từng phase, xếp thời gian tuần tự (TimerAction).
Delay để thoáng cho WSL (RTF thấp): controllers active trước khi spawn, vật + grasp_manager
sẵn sàng trước khi pick. Nếu máy chậm, tăng các mốc thời gian bên dưới.

auto_pick:=false -> chỉ dựng cảnh (gz + vật + grasp), KHÔNG tự chạy pick; gọi tay `make pick`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("mycobot_bringup")
    spawner_share = get_package_share_directory("object_spawner")
    demo_share = get_package_share_directory("mycobot_demo")
    moveit_share = get_package_share_directory("mycobot_moveit_config")

    gui = LaunchConfiguration("gui")
    num_objects = LaunchConfiguration("num_objects")
    auto_pick = LaunchConfiguration("auto_pick")

    def include(pkg_share, rel, **launch_args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", rel)),
            launch_arguments=launch_args.items(),
        )

    bringup = include(bringup_share, "bringup.launch.py", gui=gui)
    spawn = include(spawner_share, "spawn_objects.launch.py", num_objects=num_objects)
    grasp = include(demo_share, "grasp.launch.py", num_objects=num_objects)

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("num_objects", default_value="5"),
        DeclareLaunchArgument(
            "auto_pick", default_value="true",
            description="true: tự chạy pick sau khi dựng cảnh; false: chỉ dựng cảnh."),

        bringup,
        # controllers active (~8s) rồi mới spawn vật
        TimerAction(period=8.0, actions=[spawn]),
        # vật đã spawn -> grasp_manager nhả khỏi cổ tay
        TimerAction(period=12.0, actions=[grasp]),
        # cảnh sẵn sàng -> chạy pick (nếu auto_pick)
        TimerAction(period=20.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_share, "launch", "pick_and_place.launch.py")),
                launch_arguments={"num_objects": num_objects}.items(),
                condition=IfCondition(auto_pick),
            )
        ]),
    ])
