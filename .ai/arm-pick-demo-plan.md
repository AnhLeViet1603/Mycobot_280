# Plan: Cánh tay myCobot 280 gắp 5 vật thể ngẫu nhiên trong Gazebo

> Demo cá nhân — thứ 6 tuần này.
> Tham khảo: https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2/

## Bối cảnh & quyết định

| Hạng mục | Chốt |
|---|---|
| Cánh tay | myCobot 280 (6-DOF + gripper) |
| Gắp vật | Attach/detach fixed joint (plugin `DetachableJoint` của Gz8 hoặc node tự viết) |
| Điều khiển | MoveIt 2 pick-and-place |
| Nền tảng | ROS 2 Jazzy, Gazebo Sim 8, `gz_ros2_control`, `ros_gz_bridge`, MoveIt 2 |
| Vật thể | 5 cube/cylinder, kích thước random mỗi lần spawn |

**Code base cũ KHÔNG tái sử dụng:** 9 package hiện tại (conveyor, pusher, sorting, decision,
vision...) là mô hình băng chuyền đẩy piston — không liên quan cánh tay. Sẽ archive branch + xóa.

### Kinh nghiệm giữ lại từ code base cũ (đã ghi vào memory)
- `gz_ros2_control` fail spawn nếu đường dẫn file YAML params chứa chuỗi `robot_description`.
- Phải stagger (delay) việc spawn controller để tránh lỗi load controller.
- `gz sim` server còn sống ngầm sau khi tắt GUI → kill trước khi relaunch, nếu không edit bị "bỏ qua".
- Param string kiểu `y/n/on/off/yes/no` bị YAML parse thành bool → phải quote.
- Gazebo GUI cần display/GPU (WSL2 dùng WSLg); nếu GUI lỗi thì test từng node riêng.

---

## Phase 0 — Dọn dẹp & khởi tạo
1. `git branch archive/conveyor-pusher` để lưu code cũ, rồi xóa toàn bộ `src/*` cũ.
2. Cài deps: description myCobot (clone Elephant Robotics `mycobot_ros2`, nhánh gần Jazzy nhất — sẽ cần port),
   `ros_gz`, `gz_ros2_control`, `moveit`, `moveit_setup_assistant`.
3. `rosdep install` + `colcon build` khung rỗng để verify toolchain.

**Lỗi có thể gặp:** repo myCobot chỉ có nhánh Humble/Iron dùng Gazebo Classic → phải port URDF plugin
từ `libgazebo_ros_*` sang plugin `gz-sim` (`gz_ros2_control`, `JointStatePublisher`, `PosePublisher`).

## Phase 1 — Robot description (URDF/xacro)
1. Package `mycobot_description`: mesh + `mycobot_280.urdf.xacro`, kiểm tra bằng RViz.
2. Thêm `ros2_control` block (system `gz_ros2_control/GazeboSimSystem`) cho 6 joint tay + joint gripper.
3. Thêm plugin Gazebo Sim vào xacro: `gz_ros2_control-system`, `JointStatePublisher`.

**Lỗi có thể gặp:** sai đường dẫn mesh (`package://` vs `file://`); joint limit/effort = 0 khiến tay sập;
inertia không hợp lệ → robot rung/bay.

## Phase 2 — Gazebo world + bringup
1. Package `mycobot_gazebo`: `empty.world` (SDF 1.10, Gz8) + 1 bàn (table) để đặt vật.
2. `ros_gz_bridge.yaml`: bridge `/clock`, `/joint_states`, cmd controller, `/tf`.
3. `bringup.launch.py`: `gz sim` → spawn robot (`ros_gz_sim create`) → spawner controllers (stagger delay) → bridge.

**Lỗi có thể gặp:** WSL2 cần WSLg/display cho GUI; clock không sync → controller không nhận lệnh.

## Phase 3 — Controllers (ros2_control)
1. `controllers.yaml`: `joint_state_broadcaster`, `arm_group_controller` (JointTrajectoryController), `gripper_controller`.
2. Verify gửi trajectory tay thô bằng CLI trước khi lên MoveIt.

**Lỗi có thể gặp:** tên joint controller ≠ URDF; đường dẫn YAML chứa `robot_description` gây fail;
spawner chạy trước khi `/controller_manager` sẵn sàng.

## Phase 4 — MoveIt 2 config  ✅ (config viết tay, verify headless)
1. ~~MoveIt Setup Assistant~~ (headless → viết tay) → package `mycobot_moveit_config`:
   SRDF (group `arm`: g_base→grasp_center; group `gripper`; states home/ready/open/closed),
   `kinematics.yaml` (KDL), `ompl_planning.yaml`, `joint_limits.yaml`,
   `moveit_controllers.yaml` (FollowJointTrajectory → arm_controller/gripper_controller ở Phase 3),
   `.setup_assistant` (xacro use_gz:=false, mount_z:=0.42) cho MoveItConfigsBuilder.
2. `move_group.launch.py` + `moveit_rviz.launch.py` (MotionPlanning). move_group init OK headless:
   RobotModel load, KDL arm, OMPL pipeline, 2 controllers registered, capabilities loaded.
   **Còn phải test plan/execute bằng chuột trong RViz khi có display (WSLg) + bringup chạy.**
   Lệnh (3 terminal): `make gz` → `make moveit` → `make rviz-moveit`.

**Lỗi có thể gặp:** `moveit_controllers.yaml` không khớp action name của JTC; thiếu mapping ros2_control ↔ MoveIt
→ execute treo; IK không giải được với KDL default (cân nhắc trac-ik/pick-ik).

## Phase 5 — Object spawner (5 vật random)
1. Package `object_spawner`: node Python sinh 5 SDF box/cylinder kích thước random (trong khoảng gripper kẹp được),
   spawn qua service `/world/.../create`.
2. Đặt vật vị trí xác định/random trên bàn, mỗi vật tên duy nhất.

**Lỗi có thể gặp:** vật quá to/nhỏ so với gripper; spawn chồng nhau; thiếu inertia/friction → vật trượt/xuyên bàn.

## Phase 6 — Attach/detach grasp
1. Khi gripper đến gần & đóng → tạo fixed joint giữa `gripper_link` và vật (`DetachableJoint` plugin Gz8,
   điều khiển qua topic `/attach`, `/detach`), detach khi thả.
2. Node `grasp_manager` quản lý attach/detach theo tín hiệu từ pick node.

**Lỗi có thể gặp:** `DetachableJoint` cần parent/child chính xác; attach sai thời điểm → vật văng;
detach không sạch → vật dính luôn.

## Phase 7 — Pick-and-place orchestration
1. Node `pick_and_place` (moveit_py hoặc MoveGroupInterface C++): lặp 5 vật →
   approach → đóng gripper → attach → nhấc → di chuyển tới khay đích → mở gripper → detach.
2. Tính pose gắp từ vị trí + kích thước vật (chiều cao gắp theo size random).

**Lỗi có thể gặp:** pose gắp không reachable → plan fail (cần fallback/retry); thứ tự attach vs đóng gripper sai;
collision với bàn/vật khác chưa add vào planning scene.

## Phase 8 — Tích hợp & demo
1. `demo.launch.py` gộp toàn bộ; 1 lệnh cho buổi demo thứ 6.
2. Chạy thử nhiều lần với size random khác nhau; quay video dự phòng nếu GUI/WSL trục trặc.

**Lỗi có thể gặp:** timing giữa các launch; RNG seed khiến layout bất khả thi (đặt giới hạn vùng spawn).

---

## Ưu tiên cho deadline thứ 6
Đường "chắc ăn": **Phase 0→3 (arm cử động) → Phase 5 (spawn vật) → Phase 6 (attach) →
Phase 7 dùng hardcoded pose trước, thêm MoveIt (Phase 4) sau nếu còn thời gian.**
MoveIt đẹp nhưng dễ ngốn thời gian nhất; giữ bản attach + trajectory hardcode làm "phao" để demo không trắng.
