"""Cầu topic attach/detach của Gazebo sang ROS + chạy grasp_manager (Phase 6).

Cho mỗi vật obj_i: cầu 2 topic std_msgs/Empty (ROS) -> gz.msgs.Empty (GZ):
  /grasp/obj_i/attach   và   /grasp/obj_i/detach   (chiều ROS->GZ).
grasp_manager nhận tên vật trên /grasp/attach|/grasp/detach rồi bắn Empty tới topic vật đó.

Chạy SAU bringup + spawn:
  ros2 launch mycobot_demo grasp.launch.py
  ros2 launch mycobot_demo grasp.launch.py num_objects:=3
Thử tay:
  ros2 topic pub --once /grasp/attach std_msgs/msg/String "{data: obj_0}"
  ros2 topic pub --once /grasp/detach std_msgs/msg/String "{data: obj_0}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_nodes(context):
    n = int(LaunchConfiguration("num_objects").perform(context))
    prefix = LaunchConfiguration("prefix").perform(context)
    detach_on_start = LaunchConfiguration("detach_on_start").perform(context).lower() == "true"

    bridge_args = []
    for i in range(n):
        name = f"{prefix}{i}"
        # std_msgs/Empty ] gz.msgs.Empty  =>  ROS -> GZ (chỉ 1 chiều là đủ để ra lệnh)
        bridge_args.append(f"/grasp/{name}/attach@std_msgs/msg/Empty]gz.msgs.Empty")
        bridge_args.append(f"/grasp/{name}/detach@std_msgs/msg/Empty]gz.msgs.Empty")

    grasp_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="grasp_bridge",
        arguments=bridge_args,
        output="screen",
    )
    grasp_manager = Node(
        package="mycobot_demo",
        executable="grasp_manager",
        name="grasp_manager",
        output="screen",
        parameters=[{
            "num_objects": n, "prefix": prefix,
            "detach_on_start": detach_on_start, "use_sim_time": True,
        }],
    )
    return [grasp_bridge, grasp_manager]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_objects", default_value="5"),
        DeclareLaunchArgument("prefix", default_value="obj_"),
        DeclareLaunchArgument("detach_on_start", default_value="true",
                              description="nhả vật khỏi cổ tay lúc khởi động (plugin luôn attach lúc spawn)"),
        OpaqueFunction(function=_spawn_nodes),
    ])
