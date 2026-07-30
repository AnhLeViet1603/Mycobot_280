# Makefile tiện ích cho ros2_ws (myCobot 280 pick-and-place demo)
#
# Dùng --symlink-install: sửa launch/urdf/config/python KHÔNG cần build lại;
# chỉ build lại khi thêm file mới hoặc đổi CMakeLists/package.xml.
#
#   make            -> build toàn bộ (symlink)
#   make p=<pkg>    -> build nhanh 1 package (vd: make p=mycobot_description)
#   make view       -> mở RViz xem robot (Phase 1)
#   make gz         -> bringup Gazebo (Phase 2)
#   make moveit     -> move_group MoveIt 2 (Phase 4)
#   make rviz-moveit-> RViz MotionPlanning plan/execute (Phase 4)
#   make spawn      -> spawn N vật (vị trí cố định) lên bàn (Phase 5; n=<số>)
#   make grasp      -> bridge attach/detach + grasp_manager (Phase 6; n=<số vật>)
#   make pick       -> chạy pick-and-place (Phase 7; n=<số vật>)
#   make demo       -> CHẠY TOÀN BỘ 1 lệnh (Phase 8; n=<số vật>, ap=false để không tự pick)
#   make check      -> parse xacro 2 mode + check_urdf (không cần GUI)
#   make source     -> in lệnh source
#   make clean      -> xoá build/ install/ log/
#   make help       -> danh sách target

SHELL := /bin/bash
WS    := $(CURDIR)

ROS_SETUP  := /opt/ros/jazzy/setup.bash
WS_SETUP   := $(WS)/install/setup.bash
DESC_XACRO := $(WS)/src/mycobot_description/urdf/mycobot_280.urdf.xacro

p ?=

# Source ROS + workspace (nếu đã build) rồi chạy lệnh $(1)
define ros_run
source $(ROS_SETUP) && [ -f $(WS_SETUP) ] && source $(WS_SETUP); $(1)
endef

.PHONY: all build view check source clean rebuild help

## (mặc định) build toàn bộ workspace
all: build

## build: colcon build --symlink-install (p=<pkg> để build 1 package)
build:
	cd $(WS) && source $(ROS_SETUP) && \
	colcon build --symlink-install $(if $(p),--packages-select $(p),)
	@echo ">> Xong. Source:  source install/setup.bash"

## view: mở RViz xem robot (Phase 1)
view:
	@$(call ros_run, ros2 launch mycobot_description view_robot.launch.py)

## gz: bringup Gazebo có GUI (world + robot + controllers) (Phase 2)
gz:
	@$(call ros_run, ros2 launch mycobot_bringup bringup.launch.py)

## gz-test: bringup headless (không render) - test controller nhanh, RTF cao
gz-test:
	@$(call ros_run, ros2 launch mycobot_bringup bringup.launch.py gui:=false)

## test-arm: gửi vài pose cho cánh tay (Phase 3, cần Gazebo đang chạy)
test-arm:
	@$(call ros_run, ros2 run mycobot_demo test_arm)

## test-gripper: đóng rồi mở gripper (Phase 3). p=open|close để chỉ 1 chiều
test-gripper:
	@$(call ros_run, ros2 run mycobot_demo test_gripper $(if $(a),--ros-args -p action:=$(a),))

## moveit: move_group cho MoveIt 2 (Phase 4, cần Gazebo đang chạy để có /joint_states)
moveit:
	@$(call ros_run, ros2 launch mycobot_moveit_config move_group.launch.py)

## rviz-moveit: RViz MotionPlanning để plan/execute bằng chuột (cần moveit + gz đang chạy)
rviz-moveit:
	@$(call ros_run, ros2 launch mycobot_moveit_config moveit_rviz.launch.py)

## spawn: spawn N vật (vị trí cố định) lên bàn (Phase 5, cần Gazebo đang chạy). n=<số>
spawn:
	@$(call ros_run, ros2 launch object_spawner spawn_objects.launch.py \
	  $(if $(n),num_objects:=$(n),))

## grasp: bridge attach/detach + grasp_manager (Phase 6, cần bringup + spawn). n=<số vật>
grasp:
	@$(call ros_run, ros2 launch mycobot_demo grasp.launch.py $(if $(n),num_objects:=$(n),))

## pick: chạy pick-and-place (Phase 7, cần gz + spawn + grasp + moveit). n=<số vật>
pick:
	@$(call ros_run, ros2 launch mycobot_moveit_config pick_and_place.launch.py \
	  $(if $(n),num_objects:=$(n),))

## demo: CHẠY TOÀN BỘ 1 lệnh (Phase 8): gz+spawn+grasp+pick. n=<số vật>; ap=false để không tự pick
demo:
	@$(call ros_run, ros2 launch mycobot_bringup demo.launch.py \
	  $(if $(n),num_objects:=$(n),) $(if $(ap),auto_pick:=$(ap),))

## kill: dọn server gz còn sót (nếu bị kẹt)
kill:
	-pkill -f "gz sim" ; -pkill -f "ruby.*gz"

## check: parse xacro (rviz + gz) và check_urdf, không cần GUI
check:
	@$(call ros_run, \
	  echo '== use_gz:=false ==' && xacro $(DESC_XACRO) use_gz:=false > /tmp/mc_rviz.urdf && \
	  echo '== use_gz:=true =='  && xacro $(DESC_XACRO) use_gz:=true controllers_config:=/tmp/x.yaml > /tmp/mc_gz.urdf && \
	  echo '== check_urdf ==' && check_urdf /tmp/mc_rviz.urdf)

## source: in lệnh source (make không source được vào shell cha)
source:
	@echo "source $(WS_SETUP)"

## clean: xoá build/ install/ log/
clean:
	cd $(WS) && rm -rf build install log

## rebuild: clean + build
rebuild: clean build

help:
	@grep -E '^## ' $(MAKEFILE_LIST) | sed 's/## /  /'
