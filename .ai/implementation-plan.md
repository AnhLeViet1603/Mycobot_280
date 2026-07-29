# Implementation Plan — ROS 2 Conveyor Sorting Robot

> Kế hoạch triển khai chi tiết cho dự án mô tả trong [`.ai/project-idea.md`](.ai/project-idea.md).
> Chia nhỏ theo phase, mỗi phase có mục tiêu, task nhỏ, deliverable và tiêu chí hoàn thành.

## Quyết định thiết kế (đã chốt)

| Hạng mục      | Lựa chọn                                  |
| ------------- | ----------------------------------------- |
| Actuator      | **Servo pusher** (1 prismatic/revolute joint) |
| Vision        | **OpenCV HSV** color detection (YOLO để phase mở rộng) |
| Ngôn ngữ node | **Python** (`rclpy`)                      |
| OS / ROS      | Ubuntu 24.04 / ROS 2 Jazzy                |
| Simulator     | Gazebo Harmonic (`ros_gz`)                |
| Build         | colcon                                    |

## Nguyên tắc chung

- Mỗi package build được độc lập; commit sau mỗi phase.
- Ưu tiên "chạy được sớm" — có vòng lặp demo tối thiểu rồi mới tinh chỉnh.
- Test thủ công bằng `ros2 topic echo`, RViz, và Gazebo GUI sau mỗi phase.

---

## Phase 0 — Khởi tạo workspace & môi trường

**Mục tiêu:** Có workspace colcon build sạch với các package rỗng.

- [ ] Tạo cấu trúc `ros2_ws/src/` với các package:
  - `custom_interfaces` (msg) — tạo trước vì các package khác phụ thuộc
  - `conveyor_description`, `conveyor_gazebo`
  - `sorting_robot_description`, `sorting_robot_controller`
  - `object_spawner`, `vision_node`, `decision_node`
  - `bringup`
- [ ] Mỗi package Python: `package.xml`, `setup.py`, `setup.cfg`, `resource/`, `<pkg>/__init__.py`.
- [ ] `custom_interfaces` dùng `ament_cmake` (msg cần CMake).
- [ ] `colcon build` thành công, `source install/setup.bash` không lỗi.

**Deliverable:** `colcon build` xanh, `ros2 pkg list` thấy đủ package.

---

## Phase 1 — Băng chuyền (Conveyor)

**Mục tiêu:** Băng chuyền hiển thị trong Gazebo và cube trượt trên đó liên tục.

- [x] Băng chuyền (box tĩnh) định nghĩa trực tiếp trong world SDF (`conveyor` model, link `belt`).
- [x] `conveyor_gazebo/worlds/conveyor.world`: world + ánh sáng + ground plane + physics/scene systems.
- [x] Chuyển động băng: **gz-sim8 C++ system plugin `ConveyorBelt`** (port ý tưởng từ rokokoo/gazebo-conveyor) — gán vận tốc belt cho mọi vật thể nằm trên vùng băng; chỉnh "power" qua topic gz-transport.
- [x] `conveyor_gazebo/launch/conveyor.launch.py`: khởi động Gazebo với world qua `ros_gz_sim`.

> **Ghi chú:** `conveyor_description/urdf/conveyor.urdf.xacro` chưa dùng ở Phase 1 (băng đặt trực tiếp trong world). Sẽ bổ sung khi cần robot_description/RViz.

**Deliverable:** Đặt 1 cube lên băng → cube di chuyển liên tục về cuối băng.

**Tiêu chí:** `ros2 launch conveyor_gazebo conveyor.launch.py` mở Gazebo có băng chuyền, cube trôi ổn định.

---

## Phase 2 — Spawn vật thể

**Mục tiêu:** Cube màu ngẫu nhiên (đỏ/xanh dương/xanh lá) xuất hiện định kỳ.

- [x] `object_spawner/object_spawner/spawner_node.py`: node gọi thẳng service `/world/<world>/create` của Gazebo qua **gz-transport Python** (không cần bridge).
- [x] Sinh cube SDF theo màu random (đỏ/xanh dương/xanh lá) ở đầu băng; băng plugin lo phần chuyển động.
- [x] Param: `spawn_period`, `color_weights`, `cube_size`, `spawn_x/y/z`, `jitter_y`.
- [x] Launch file `spawner.launch.py` để test độc lập.

**Deliverable:** Cube màu ngẫu nhiên tự xuất hiện mỗi vài giây và chạy trên băng.

**Tiêu chí:** Chạy spawner → thấy cube đỏ/xanh/lục lần lượt trôi qua.

---

## Phase 3 — Camera

**Mục tiêu:** Camera Gazebo publish ảnh, xem được trong RViz.

- [x] Sensor camera đặt phía trên băng (nhìn xuống), thêm `gz-sim-sensors-system` vào world.
- [x] Bridge `ros_gz_bridge` (config `config/bridge.yaml`): `/camera/image_raw` (rgb8 640×480), `/camera/camera_info`.
- [x] RViz config `config/conveyor.rviz` hiển thị Image; bật qua `rviz:=true` trong `conveyor.launch.py`.

**Deliverable:** Luồng camera trực tiếp trong RViz.

**Tiêu chí:** `ros2 topic hz /camera/image_raw` > 0, thấy cube trong khung hình.

---

## Phase 4 — Nhận diện màu (OpenCV HSV)

**Mục tiêu:** Từ ảnh camera phát hiện màu cube và vị trí.

- [x] `custom_interfaces/msg/DetectedObject.msg`: `class_name` + `geometry_msgs/Point position` (đã có từ Phase 0).
- [x] `vision_node/vision_node.py`: sub `/camera/image_raw`, `cv_bridge` + HSV threshold, tìm contour lớn nhất, phân loại màu.
- [x] Publish `/detected_object` (`DetectedObject`): x,y = tâm blob (pixel), z = diện tích blob.
- [x] Param: `min_area`, `image_topic`, `publish_debug` (ngưỡng HSV trong `HSV_RANGES`).
- [x] Publish ảnh debug `/vision/debug_image` có contour + nhãn màu.

**Deliverable:** Topic `/detected_object` phát class + vị trí đúng.

**Tiêu chí:** Cho cube đỏ đi qua → `class_name: red`; tương tự blue/green.

---

## Phase 5 — Node quyết định

**Mục tiêu:** Quyết định giữ hay loại dựa trên màu.

- [x] `decision_node/decision_node/decision_node.py`: sub `/detected_object`.
- [x] Logic: `blue → KEEP`, còn lại `→ REJECT`.
- [x] Publish `/reject_object` (dùng `DetectedObject` để kèm màu + vị trí).
- [x] Param: `accepted_color` (mặc định `blue`) để dễ đổi.
- [x] Chống trigger lặp cho cùng một cube (debounce theo thời gian, `debounce_sec`).
- [x] Despawn cube ở cuối băng: `object_spawner` xoá model qua service `/world/<world>/remove` sau `cube_lifetime` giây.

**Deliverable:** Thông điệp quyết định trên `/reject_object`.

**Tiêu chí:** Chỉ cube không phải blue mới tạo lệnh reject.

---

## Phase 6 — Servo pusher (URDF + Gazebo)

**Mục tiêu:** Cơ cấu đẩy đặt cạnh băng, có joint điều khiển được.

- [x] `sorting_robot_description/urdf/pusher.urdf.xacro`: base cố định vào `world` + 1 khớp prismatic đẩy ngang (trục Y).
- [x] Thêm `<ros2_control>` tag (`gz_ros2_control/GazeboSimSystem`) + plugin `gz_ros2_control-system`.
- [x] Spawn pusher cạnh cuối băng (x≈0.6, y≈0.5), paddle quét về -Y để đẩy cube rơi khỏi băng.
- [x] Launch `pusher.launch.py` (+ `config/pusher_controllers.yaml`): world + robot_state_publisher + spawn + `joint_state_broadcaster` + `pusher_position_controller` (forward_command).

**Deliverable:** Pusher hiển thị trong Gazebo, joint di chuyển được (test bằng tay).

**Tiêu chí:** Publish thử joint command → thanh đẩy duỗi/thu.

---

## Phase 7 — Điều khiển servo (ros2_control)

**Mục tiêu:** Điều khiển pusher theo lệnh reject, có state machine.

- [x] `sorting_robot_controller/config/pusher_controllers.yaml`: `forward_command_controller` (position) + `joint_state_broadcaster` (tái dùng từ Phase 6).
- [x] `sorting_robot_controller/sorting_robot_controller/pusher_controller_node.py`:
  - Sub `/reject_object`.
  - State machine: `IDLE → WAIT → EXTEND → HOLD → RETRACT → IDLE` (one-shot timers).
  - Gửi `Float64MultiArray` tới `/pusher_position_controller/commands`; `trigger_delay` căn thời gian đẩy khớp lúc cube tới pusher.
- [x] Param: `extend_position`, `retract_position`, `trigger_delay`, `extend_time`, `hold_time`, `retract_time`.
- [x] Sub `/joint_states`; command topic controller hoạt động (kiểm chứng bằng reject giả → phát -0.6 rồi 0.0).
- [x] Launch `pusher_control.launch.py`: gộp `pusher.launch.py` (Phase 6) + `pusher_controller_node`.

**Deliverable:** Nhận lệnh reject → pusher đẩy cube văng khỏi băng rồi thu về.

**Tiêu chí:** Cube đỏ/lục bị đẩy ra; pusher trở lại IDLE.

---

## Phase 8 — Tích hợp toàn bộ (bringup)

**Mục tiêu:** Một launch chạy toàn bộ pipeline.

- [x] `bringup/launch/system.launch.py`: Gazebo + conveyor + camera + bridge + spawner + vision + decision + controller + RViz (staggered TimerAction: world → nodes 3s → pusher 6s → spawner 15s).
- [x] File param tập trung trong `bringup/config/system_params.yaml` (áp cho object_spawner, vision_node, decision_node, pusher_controller_node — đã kiểm chứng params nạp đúng).
- [x] Điều chỉnh timing end-to-end (spawn → detect → decide → push) qua param `trigger_delay` + các TimerAction (cần chạy Gazebo có màn hình để tinh chỉnh cuối).
- [x] README hướng dẫn chạy demo (`ros2 launch bringup system.launch.py`).

**Deliverable:** Demo hoàn chỉnh:
- Blue cube → đi hết băng (KEEP).
- Red/Green cube → bị pusher đẩy loại (REJECT).

**Tiêu chí:** `ros2 launch bringup system.launch.py` chạy trọn pipeline không thao tác thủ công.

---

## Phase 9 (Tùy chọn) — Mở rộng

- [ ] Thay OpenCV bằng **YOLOv8** (giữ nguyên interface `DetectedObject`).
- [ ] Dashboard đếm accepted/rejected + throughput.
- [ ] Logging SQLite (timestamp, class, kết quả).
- [ ] Phân loại theo shape/size, phát hiện lỗi sản phẩm.
- [ ] Nâng cấp servo pusher → cánh tay robot (UR5/Panda + MoveIt).

---

## Thứ tự phụ thuộc

```
Phase 0
  └─ custom_interfaces (msg) ─────────────┐
Phase 1 (conveyor) ─ Phase 2 (spawn) ─ Phase 3 (camera)
                                            └─ Phase 4 (vision) ─ Phase 5 (decision)
Phase 6 (pusher urdf) ─ Phase 7 (control) ──────────────────────┘
                                            └─ Phase 8 (bringup, tích hợp tất cả)
```

## Checklist môi trường trước khi bắt đầu

- [ ] ROS 2 Jazzy đã cài, `ros2` chạy được.
- [ ] Gazebo Harmonic + `ros_gz`, `gz_ros2_control` đã cài.
- [ ] `ros2_control`, `ros2_controllers`, `cv_bridge`, `python3-opencv` đã cài.
- [ ] `colcon`, `rosdep` sẵn sàng.
