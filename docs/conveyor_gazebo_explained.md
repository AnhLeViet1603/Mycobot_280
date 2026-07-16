# Giải thích chi tiết module `conveyor_gazebo`

> Package này dựng **thế giới mô phỏng**: băng chuyền, camera trên cao, và một
> **plugin C++ tự viết** (`ConveyorBelt`) để làm băng chuyền thực sự "chạy". Đây
> là package **`ament_cmake`** vì có mã C++ biên dịch thành thư viện plugin.

---

## 1. Vai trò & cấu trúc

```
src/conveyor_gazebo/
├── worlds/conveyor.world        # Mô tả thế giới Gazebo (SDF)
├── src/ConveyorBelt.cc          # Plugin C++ đẩy vật thể trên băng chuyền
├── include/conveyor_belt/ConveyorBelt.hh
├── config/
│   ├── bridge.yaml              # Cấu hình cầu nối Gazebo -> ROS 2
│   └── conveyor.rviz            # Layout RViz để xem camera
├── hooks/conveyor_gazebo.dsv.in # Hook môi trường để Gazebo tìm thấy plugin
├── launch/conveyor.launch.py    # Khởi động Gazebo + bridge + (RViz)
├── CMakeLists.txt
└── package.xml
```

---

## 2. `worlds/conveyor.world` — Thế giới mô phỏng (SDF)

Định dạng **SDF** (Simulation Description Format) mô tả toàn bộ scene.

### Các khối chính

- **`<physics>` + các `<plugin>` hệ thống**: bật engine vật lý, xử lý lệnh người
  dùng, phát scene, và **sensor** (dòng 16–19). Plugin `Sensors` với
  `<render_engine>ogre2</render_engine>` là thứ cho camera render được ảnh — cần GPU/display.

- **`ground_plane`** và **`sun`**: mặt đất tĩnh và nguồn sáng directional.

- **`conveyor`** (băng chuyền) — dòng 51–85:
  ```xml
  <model name="conveyor">
    <static>true</static>
    <pose>0 0 0.5 0 0 0</pose>
    <link name="belt"> ... box 2.0 x 0.5 x 0.1 ... </link>
    <plugin filename="ConveyorBelt" name="conveyor_belt::ConveyorBelt">
      <belt_link>belt</belt_link>
      <velocity>0.3</velocity>   <!-- m/s -->
      <length>2.0</length> <width>0.5</width> <height>0.3</height>
    </plugin>
  </model>
  ```
  - `<static>true</static>`: bản thân slab băng chuyền **không di chuyển** — nó chỉ
    là mặt phẳng. Chuyển động của cube do **plugin** áp vận tốc, không phải slab quay.
  - `<plugin filename="ConveyorBelt">`: gắn plugin C++ tự viết vào model này. Các
    tham số bên trong (`velocity`, `length`...) được plugin đọc lúc `Configure`.

- **`camera`** (camera trên cao) — dòng 87–114:
  ```xml
  <model name="camera">
    <pose>0.6 0 1.5 0 1.5708 0</pose>   <!-- ngay trên pusher (x=0.6), nhìn thẳng xuống -->
    <sensor name="camera" type="camera">
      <topic>camera</topic>
      <update_rate>15</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
      </camera>
    </sensor>
  </model>
  ```
  - **`<pose>0.6 0 1.5 0 1.5708 0</pose>`**: đặt camera tại `x=0.6` (ngay trên
    pusher), cao 1.5m; `pitch = 1.5708 rad ≈ 90°` để **nhìn thẳng xuống**. Đây là
    lý do vision detect cube đúng ngay vị trí paddle → giảm khoảng trễ giữa "thấy" và "đẩy".
  - **`<topic>camera</topic>`**: tên topic phía Gazebo — sẽ được bridge ánh xạ sang ROS 2.

---

## 3. `src/ConveyorBelt.cc` — Plugin băng chuyền (C++)

Plugin là một **System plugin** của Gazebo Sim, chạy mỗi bước mô phỏng. Nó dùng
mẫu **PImpl** (`class Impl`) để giấu chi tiết.

### `Configure()` — chạy MỘT lần khi plugin nạp

- Đọc các tham số từ SDF: `belt_link`, `velocity`, `length`, `width`, `height`
  (dùng `_sdf->Get<T>("tên", mặc_định)`).
- Tính `halfLength = length/2`, `halfWidth = width/2` để so sánh vùng băng chuyền.
- Tìm entity của link băng chuyền; nếu không có → in lỗi và tắt plugin.
- Đăng ký một topic **power** cho phép chỉnh tốc độ băng chuyền lúc chạy (0–100%).

### `PreUpdate()` — chạy MỖI bước mô phỏng (phần lõi)

```cpp
const gz::math::Vector3d forward = beltPose.Rot().RotateVector(UnitX);
const gz::math::Vector3d beltVel = forward * (velocity * power);
```
- Tính **hướng tiến của băng chuyền** (+X của link) trong hệ tọa độ thế giới, rồi
  nhân với tốc độ và hệ số công suất → vector vận tốc cần áp.

```cpp
_ecm.Each<Model, Name, Pose>([&](...) {
    // bỏ qua chính băng chuyền và các model static (ground, camera...)
    // đưa vị trí model về hệ tọa độ băng chuyền:
    const auto local = beltPose.Inverse().CoordPositionAdd(_pose->Data().Pos());
    // nếu nằm ngoài vùng slab -> bỏ qua
    if (abs(local.X()) > halfLength || abs(local.Y()) > halfWidth ||
        local.Z() < -0.05 || local.Z() > height) return true;
    // nếu đang nằm trên băng chuyền -> gán vận tốc cho canonical link của model
    _ecm.CreateComponent(canonical, LinearVelocityCmd(beltVel)); // hoặc gán lại
});
```

**Ý nghĩa:** mỗi bước, plugin duyệt **mọi model** trong world, xác định model nào
đang **nằm trên bề mặt băng chuyền** (kiểm tra vị trí trong hệ tọa độ băng chuyền
nằm trong hộp giới hạn), rồi **áp trực tiếp một lệnh vận tốc** (`LinearVelocityCmd`)
lên link chính của model đó. Kết quả: cube "được đẩy" theo băng chuyền mà không cần
băng chuyền thật sự quay.

- **Bỏ qua model `static`** (dòng 73–81): tránh áp vận tốc lên ground/camera.
- **`beltPose.Inverse().CoordPositionAdd(...)`**: chuyển tọa độ điểm từ hệ thế giới
  về hệ băng chuyền để kiểm tra "có nằm trong vùng slab không" một cách đơn giản.

### Đăng ký plugin

```cpp
GZ_ADD_PLUGIN(conveyor_belt::ConveyorBelt, gz::sim::System,
    ISystemConfigure, ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(conveyor_belt::ConveyorBelt, "conveyor_belt::ConveyorBelt")
```
- Macro báo Gazebo rằng class này là một System có 2 giao diện: `Configure` và
  `PreUpdate`. Alias cho phép tham chiếu bằng tên trong file `.world`.

---

## 4. `config/bridge.yaml` — Cầu nối Gazebo ↔ ROS 2

```yaml
- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/camera"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
- ros_topic_name: "/camera/camera_info"
  gz_topic_name: "/camera_info"
  ...
```
- Mỗi mục ánh xạ **một topic Gazebo ↔ một topic ROS 2**, kèm kiểu message hai bên và **hướng** truyền.
- `GZ_TO_ROS`: dữ liệu chỉ chảy từ Gazebo sang ROS 2 (camera là nguồn, ROS 2 tiêu thụ).
- Nhờ file này, `vision_node` chỉ cần subscribe `/camera/image_raw` như một topic ROS 2 bình thường.

---

## 5. `launch/conveyor.launch.py`

Khởi động 3 thứ:

1. **`gz_sim`** — nhúng launch chuẩn của `ros_gz_sim`:
   ```python
   launch_arguments={'gz_args': [world, ' -r -v 3']}.items()
   ```
   - `-r`: chạy ngay (không pause). `-v 3`: log mức info. `world`: đường dẫn file `.world`
     (mặc định trỏ tới `conveyor.world`, có thể override qua `world:=...`).

2. **`bridge`** — node `parameter_bridge` của `ros_gz_bridge`, nạp `bridge.yaml`
   để bắc cầu camera.

3. **`rviz_node`** — chỉ chạy khi `rviz:=true` nhờ `condition=IfCondition(rviz)`.
   `IfCondition` biến chuỗi `"true"/"false"` thành điều kiện bật/tắt action.

---

## 6. `CMakeLists.txt` — điểm đáng chú ý

```cmake
add_library(ConveyorBelt SHARED src/ConveyorBelt.cc)
target_link_libraries(ConveyorBelt gz-sim8::... gz-transport13::... gz-msgs10::...)
install(TARGETS ConveyorBelt LIBRARY DESTINATION lib ...)
install(DIRECTORY worlds launch config DESTINATION share/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
```
- **`add_library(... SHARED ...)`**: biên dịch plugin thành thư viện động (`.so`).
- **`install(TARGETS ...)`**: copy `.so` vào `lib/` để Gazebo nạp được.
- **`install(DIRECTORY worlds launch config ...)`**: copy tài nguyên (tương đương
  `data_files` bên ament_python).
- **`ament_environment_hooks(...)`** ⭐: đăng ký một hook thiết lập biến môi trường
  (`GZ_SIM_SYSTEM_PLUGIN_PATH`) khi `source install/setup.bash`, để Gazebo **tìm
  thấy** plugin `ConveyorBelt`. Thiếu hook này → world báo "không nạp được plugin".

---

## 7. Lưu ý vận hành

- Camera cần **render engine (ogre2)** → cần display/GPU. Trong container không có
  GPU, phần camera có thể không tạo ảnh (xem ghi chú dự án về chạy headless).
- Nếu sửa `.world` mà thấy "không có tác dụng", hãy **kill tiến trình `gz sim` cũ**
  còn sót trước khi chạy lại (ghi chú dự án đã lưu điều này).
