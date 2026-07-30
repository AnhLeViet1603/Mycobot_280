cmake_minimum_required(VERSION 3.22)
project(mycobot_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# .setup_assistant giúp MoveItConfigsBuilder định vị URDF/SRDF khi chạy launch.
install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
