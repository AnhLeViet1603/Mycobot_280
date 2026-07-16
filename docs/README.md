# Tài liệu giải thích code — Robot phân loại cube trên băng chuyền

Bộ tài liệu này giải thích chi tiết **từng package** trong workspace ROS 2: vai trò
của từng file và ý nghĩa các đoạn code phức tạp. Tất cả viết bằng tiếng Việt.

## Tổng quan hệ thống

Demo mô phỏng một robot loại bỏ cube lỗi trên băng chuyền. Luồng end-to-end:

```
object_spawner  → sinh cube màu ở đầu băng chuyền (gọi thẳng service Gazebo)
      │
conveyor_gazebo → thế giới Gazebo: băng chuyền (plugin C++) + camera trên cao
      │  (ros_gz_bridge: /camera -> /camera/image_raw)
vision_node     → OpenCV HSV, tìm cube trong ROI  → /detected_object
      │
decision_node   → blue = GIỮ, còn lại = LOẠI       → /reject_object
      │
sorting_robot_controller → state machine đẩy      → /pusher_position_controller/commands
      │  (ros2_control)
sorting_robot_description → robot pusher (URDF/xacro) trong Gazebo
      │
      ▼
   Cube lỗi bị paddle đẩy rơi khỏi băng chuyền
```

Tất cả khởi động bằng một lệnh: `ros2 launch bringup system.launch.py`.

## Mục lục tài liệu

| Package | Loại build | Vai trò | Tài liệu |
|---------|-----------|---------|----------|
| `bringup` | ament_python | Launch tổng + tham số tập trung ("nhạc trưởng") | [bringup_module_explained.md](bringup_module_explained.md) |
| `custom_interfaces` | ament_cmake | Message `DetectedObject` dùng chung | [custom_interfaces_explained.md](custom_interfaces_explained.md) |
| `conveyor_gazebo` | ament_cmake | Thế giới Gazebo + plugin băng chuyền C++ + bridge | [conveyor_gazebo_explained.md](conveyor_gazebo_explained.md) |
| `object_spawner` | ament_python | Sinh/xóa cube qua service Gazebo | [object_spawner_explained.md](object_spawner_explained.md) |
| `vision_node` | ament_python | Nhận diện màu bằng OpenCV HSV | [vision_node_explained.md](vision_node_explained.md) |
| `decision_node` | ament_python | Chính sách giữ/loại + debounce | [decision_node_explained.md](decision_node_explained.md) |
| `sorting_robot_description` | ament_python | URDF pusher + spawn + ros2_control | [sorting_robot_description_explained.md](sorting_robot_description_explained.md) |
| `sorting_robot_controller` | ament_python | State machine đẩy + cấu hình controller | [sorting_robot_controller_explained.md](sorting_robot_controller_explained.md) |
| `conveyor_description` | ament_python | Placeholder (chưa dùng) | [conveyor_description_explained.md](conveyor_description_explained.md) |

## Các "bẫy" chung của dự án (đọc nhanh)

1. **Đường dẫn chứa `robot_description`** làm hỏng `--params-file` của
   controller_manager → YAML controller phải để ở `sorting_robot_controller`.
2. **YAML boolean trap**: `y/n/on/off/yes/no` bị hiểu thành bool → luôn đặt ngoặc kép
   cho giá trị chuỗi (ví dụ `trigger_axis: "y"`).
3. **Quên khai báo `launch/`, `config/` trong `data_files`/`install`** → file không
   sang `install/`, `ros2 launch` báo không tìm thấy.
4. **Staggered startup**: các thành phần cần khởi động đúng thứ tự/độ trễ (Gazebo →
   node → spawn robot → controller → spawner).
5. **Camera cần GPU/display** (render ogre2) → khó chạy headless trong container.
6. **Kill `gz sim` cũ** trước khi chạy lại nếu chỉnh sửa không có tác dụng.
