# ROS 2 Conveyor Sorting Robot Simulation

Mô phỏng dây chuyền phân loại vật thể công nghiệp bằng **ROS 2 + Gazebo**: cube màu chạy trên băng chuyền, camera + OpenCV nhận diện màu, node quyết định phân loại, và một **servo pusher** đẩy loại các cube không hợp lệ.

Kịch bản: nhà máy chỉ nhận **cube xanh dương** — cube đỏ và xanh lá bị đẩy khỏi băng.

```
Spawner → Conveyor → Camera → Vision (OpenCV) → Decision → Servo Pusher
```

## Tech stack

| Thành phần | Công nghệ            |
| ---------- | ------------------- |
| OS         | Ubuntu 24.04        |
| ROS        | ROS 2 Jazzy         |
| Simulator  | Gazebo Harmonic     |
| Robot      | URDF + ros2_control |
| Vision     | OpenCV (HSV)        |
| Ngôn ngữ   | Python (`rclpy`)    |
| Build      | colcon              |

## Cấu trúc workspace

```
ros2_ws/src/
  custom_interfaces/          # msg (DetectedObject, ...)
  conveyor_description/       # URDF băng chuyền
  conveyor_gazebo/            # world + launch Gazebo
  sorting_robot_description/  # URDF servo pusher
  sorting_robot_controller/   # điều khiển pusher (ros2_control)
  object_spawner/             # spawn cube màu ngẫu nhiên
  vision_node/                # nhận diện màu OpenCV
  decision_node/              # logic keep/reject
  bringup/                    # launch tích hợp toàn hệ thống
```

## Yêu cầu

Cài ROS 2 Jazzy, Gazebo Harmonic và các package phụ thuộc:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
  ros-jazzy-cv-bridge python3-opencv
```

## Build

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Chạy demo

```bash
ros2 launch bringup system.launch.py
```

Kết quả mong đợi:
- **Cube xanh dương** → đi hết băng chuyền (giữ lại).
- **Cube đỏ / xanh lá** → bị servo pusher đẩy ra khỏi băng (loại).

## Lộ trình phát triển

Xem [`implementation-plan.md`](implementation-plan.md) để biết chi tiết từng phase (Phase 0 → 9).

## Ý tưởng gốc

Xem [`.ai/project-idea.md`](.ai/project-idea.md).
