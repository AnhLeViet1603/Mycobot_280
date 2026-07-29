# Tìm hiểu Gazebo + ROS 2 và Demo Robot Simulation
> Thời lượng: **10 phút**
>
> Mục tiêu:
> - Giới thiệu Gazebo và ROS 2.
> - Giải thích cách hai nền tảng phối hợp với nhau.
> - Trình diễn một demo mô phỏng robot.
> - Cho thấy khả năng mở rộng sang các ứng dụng thực tế.

---

# 1. Mục tiêu buổi trình bày (30s)

## Nội dung

- Gazebo là gì?
- ROS 2 là gì?
- Gazebo và ROS 2 hoạt động cùng nhau như thế nào?
- Demo một hệ thống robot mô phỏng.
- Hướng phát triển sau demo.

---

# 2. Gazebo là gì? (1 phút)

## Khái niệm

Gazebo là phần mềm mô phỏng robot 3D mã nguồn mở.

Cho phép mô phỏng:

- Robot
- Cảm biến
- Camera
- LiDAR
- IMU
- Servo
- Động cơ
- Môi trường vật lý

## Vai trò

Thay vì chạy trực tiếp trên robot thật:

```
Viết chương trình
        ↓
Chạy trên Gazebo
        ↓
Kiểm tra
        ↓
Đưa sang robot thật
```

## Hình minh họa

- Giao diện Gazebo
- Robot trong môi trường 3D

---

# 3. ROS 2 là gì? (1 phút)

## ROS 2

Robot Operating System 2 là framework phát triển phần mềm robot.

Không phải hệ điều hành.

ROS 2 cung cấp:

- Node
- Topic
- Service
- Action
- Parameter
- TF
- Launch

## Kiến trúc

```
Camera Node
      │
      ▼
 Image Topic
      │
      ▼
 Vision Node
      │
      ▼
 Detection Topic
      │
      ▼
 Controller Node
      │
      ▼
 Robot
```

---

# 4. Gazebo + ROS 2 hoạt động như thế nào? (1 phút)

## Mối quan hệ

Gazebo:

- Mô phỏng thế giới

ROS 2:

- Điều khiển robot

Hai bên kết nối bằng:

```
ros_gz
```

## Luồng dữ liệu

```
Gazebo
   │
Camera
LiDAR
Joint
   │
ros_gz_bridge
   │
ROS 2
   │
Controller
   │
Command
   │
Gazebo
```

Điểm quan trọng:

- Gazebo tạo dữ liệu cảm biến.
- ROS 2 xử lý dữ liệu.
- ROS 2 gửi lệnh điều khiển trở lại Gazebo.

---

# 5. Demo (4 phút)

## Đề tài

Robot trên băng chuyền loại bỏ sản phẩm lỗi.

Ví dụ:

- Cube đỏ → lỗi
- Cube xanh → đạt

Robot sẽ đẩy cube đỏ khỏi băng chuyền.

---

## Kiến trúc Demo

```
Camera

      │

Gazebo

      │

ROS 2

      │

Nhận diện màu

      │

Servo Pusher

      │

Đẩy cube đỏ
```

---

## Thành phần

Gazebo

- Conveyor
- Cube
- Servo
- Camera

ROS 2

- Image Subscriber
- Color Detection
- Controller

---

## Luồng hoạt động

```
Cube chạy trên băng chuyền

↓

Camera quan sát

↓

ROS2 nhận ảnh

↓

Phân loại màu

↓

Nếu đỏ

↓

Servo đẩy

↓

Cube rơi khỏi băng chuyền
```

---

## Video hoặc Demo Live

Có thể trình diễn:

- Khởi động Gazebo
- Spawn robot
- Chạy conveyor
- Cube xuất hiện
- Servo đẩy cube đỏ

Nếu không demo trực tiếp:

Chuẩn bị video 30–45 giây.

---

# 6. Gazebo trong thực tế (1 phút)

Gazebo thường được dùng trong:

- Robot công nghiệp
- Robot kho hàng
- Robot tự hành (AGV/AMR)
- Drone
- Robot nghiên cứu

Ví dụ:

- Kiểm thử thuật toán trước khi chạy robot thật.
- Phát triển hệ thống AI nhận diện.
- Kiểm tra chuyển động robot.

---

# 7. Kết luận (30s)

## Tổng kết

Gazebo

✔ Mô phỏng môi trường

ROS 2

✔ Điều khiển robot

Kết hợp:

✔ Tiết kiệm chi phí

✔ Phát triển nhanh

✔ An toàn

✔ Có thể chuyển sang robot thật

---

# 8. Hướng phát triển

Demo hiện tại mới sử dụng:

- Camera
- Conveyor
- Servo

Có thể mở rộng:

- YOLO nhận diện vật thể
- Robot Arm gắp hàng
- Mobile Robot
- AGV
- Navigation
- SLAM
- AI Vision
- Digital Twin

---

# Phân bổ thời gian

| Phần | Thời gian |
|-------|-----------|
| Mục tiêu | 0.5 phút |
| Gazebo | 1 phút |
| ROS 2 | 1 phút |
| Gazebo + ROS 2 | 1 phút |
| Demo | 4 phút |
| Ứng dụng | 1 phút |
| Kết luận | 0.5 phút |

Tổng: **10 phút**

---

# Gợi ý Slide

Slide 1
- Tiêu đề
- Tên đề tài

Slide 2
- Agenda

Slide 3
- Gazebo

Slide 4
- ROS 2

Slide 5
- Gazebo + ROS 2 Architecture

Slide 6
- Demo Architecture

Slide 7
- Demo Live / Video

Slide 8
- Ứng dụng thực tế

Slide 9
- Kết luận & Hướng phát triển

---

# Một số lưu ý khi thuyết trình

- Không đi sâu vào API hay code ROS 2.
- Dùng nhiều sơ đồ hơn là chữ.
- Demo nên chiếm khoảng 40% thời lượng (4/10 phút) để tạo điểm nhấn.
- Chuẩn bị sẵn video dự phòng phòng trường hợp Gazebo hoặc ROS 2 khởi động chậm.
- Trong phần demo, tập trung giải thích luồng xử lý (camera → ROS 2 → điều khiển → Gazebo) thay vì mô tả từng dòng lệnh.