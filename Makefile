# Makefile tiện ích cho ros2_ws (conveyor sorting pipeline)
#
#   make build     -> colcon build + hướng dẫn source
#   make source    -> in lệnh source (chạy: source install/setup.bash)
#   make run       -> full pipeline + RViz
#   make gazebo    -> chỉ Gazebo world (conveyor)
#   make push      -> kích hoạt (đẩy ra) pusher
#   make retract   -> thu hồi (kéo về) pusher
#   make clean     -> xoá build/ install/ log/

# Cho phép ROS/colcon dùng bash và source được setup
SHELL := /bin/bash
WS     := $(CURDIR)

# Topic điều khiển pusher (forward_command_controller)
PUSH_TOPIC   := /pusher_position_controller/commands
PUSH_EXTEND  := -0.68   # vị trí đẩy ra hết hành trình (theo system_params.yaml)
PUSH_RETRACT := 0.0     # vị trí thu về

.PHONY: build source run gazebo push retract clean help

## build: colcon build rồi nhắc source
build:
	cd $(WS) && colcon build --symlink-install
	@echo ""
	@echo ">> Build xong. Chạy tiếp:  source install/setup.bash"

## source: in lệnh source (make không thể source vào shell cha của bạn)
source:
	@echo "source $(WS)/install/setup.bash"

## run: full pipeline + RViz
run:
	source /opt/ros/*/setup.bash && source $(WS)/install/setup.bash && \
		ros2 launch bringup system.launch.py rviz:=true

## gazebo: chỉ chạy Gazebo world (conveyor + camera bridge)
gazebo:
	source /opt/ros/*/setup.bash && source $(WS)/install/setup.bash && \
		ros2 launch conveyor_gazebo conveyor.launch.py

## push: kích hoạt pusher (đẩy cube ra)
push:
	source /opt/ros/*/setup.bash && source $(WS)/install/setup.bash && \
		ros2 topic pub --once $(PUSH_TOPIC) std_msgs/msg/Float64MultiArray \
		"{data: [$(PUSH_EXTEND)]}"

## retract: thu hồi pusher (kéo về)
retract:
	source /opt/ros/*/setup.bash && source $(WS)/install/setup.bash && \
		ros2 topic pub --once $(PUSH_TOPIC) std_msgs/msg/Float64MultiArray \
		"{data: [$(PUSH_RETRACT)]}"

## clean: xoá artifact build
clean:
	cd $(WS) && rm -rf build install log

help:
	@grep -E '^## ' $(MAKEFILE_LIST) | sed 's/## /  /'
