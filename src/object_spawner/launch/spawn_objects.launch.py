"""Cầu service create của Gazebo sang ROS rồi spawn N vật random lên bàn.

Chạy SAU khi bringup.launch.py đã lên (Gazebo + world pick_world đang chạy):
  ros2 launch object_spawner spawn_objects.launch.py
  ros2 launch object_spawner spawn_objects.launch.py num_objects:=3 seed:=42
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    world = LaunchConfiguration("world")
    num_objects = LaunchConfiguration("num_objects")
    seed = LaunchConfiguration("seed")

    # Cầu service Gazebo /world/<world>/create -> ROS (chỉ ROS->GZ, forward request).
    create_service = PythonExpression(
        ["'/world/' + '", world, "' + '/create@ros_gz_interfaces/srv/SpawnEntity'"]
    )
    create_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="create_service_bridge",
        arguments=[create_service],
        output="screen",
    )

    spawner = Node(
        package="object_spawner",
        executable="spawn_objects",
        name="object_spawner",
        output="screen",
        parameters=[{
            "world": world,
            "num_objects": ParameterValue(num_objects, value_type=int),
            "seed": ParameterValue(seed, value_type=int),
            "use_sim_time": True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="pick_world"),
        DeclareLaunchArgument("num_objects", default_value="5"),
        DeclareLaunchArgument("seed", default_value="-1",
                              description=">=0 để tái lập layout; <0 = random thật"),
        create_bridge,
        spawner,
    ])
