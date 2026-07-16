# Giải thích chi tiết module `sorting_robot_description`

> Package **mô tả robot pusher** (URDF/xacro) và launch file để **spawn robot vào
> Gazebo + khởi động `ros2_control`**. Đây là nơi định nghĩa phần cứng: một cột
> tĩnh cạnh băng chuyền với một khớp trượt (prismatic) mang paddle đẩy cube.

---

## 1. Vai trò & cấu trúc

```
src/sorting_robot_description/
├── urdf/pusher.urdf.xacro       # Mô tả robot + cấu hình ros2_control
├── launch/pusher.launch.py      # Spawn robot + nạp controller (staggered)
├── setup.py
└── package.xml
```

---

## 2. `urdf/pusher.urdf.xacro` — Mô tả robot

Dùng **xacro** (URDF có macro/biến) để mô tả robot.

### 2.1. Tham số & biến

```xml
<xacro:arg name="controllers_config" default=""/>   <!-- đường dẫn YAML, launch truyền vào -->
<xacro:property name="base_x" value="0.6"/>          <!-- vị trí đặt cột (ngay dưới camera) -->
<xacro:property name="base_y" value="0.35"/>
<xacro:property name="base_z" value="0.615"/>
<xacro:property name="stroke" value="0.70"/>         <!-- hành trình khớp trượt (m) -->
```
- **`<xacro:arg>`**: đối số truyền từ ngoài (launch file) vào — ở đây là đường dẫn
  file cấu hình controller.
- **`<xacro:property>`**: hằng số nội bộ; đặt một chỗ, dùng lại bằng `${tên}`.

### 2.2. Chuỗi link/joint

```
world (link ảo cố định)
  └─[world_joint: fixed]→ base_link (cột tĩnh)
        └─[pusher_joint: prismatic, trục Y]→ pusher_link (paddle)
```

- **`world` + `world_joint` (fixed)**: link `world` là mốc cố định; khớp fixed
  **neo** `base_link` vào đúng vị trí `(base_x, base_y, base_z)` trong Gazebo →
  robot không bị rơi/trôi.
- **`pusher_joint` (prismatic)** — khớp trượt, phần quan trọng nhất:
  ```xml
  <axis xyz="0 1 0"/>
  <limit lower="-${stroke}" upper="0.0" effort="300.0" velocity="3.0"/>
  <dynamics damping="0.05" friction="0.0"/>
  ```
  - Trượt dọc trục **Y**. `upper=0.0` = thu về (thoát băng chuyền), `lower=-0.70` =
    duỗi hết ngang băng chuyền (âm vì trượt về phía -Y).
  - **`effort="300.0"`, `velocity="3.0"`** cao có chủ đích (xem comment): để lực
    cản của cube không đáng kể và cú trượt **luôn hoàn tất trong `extend_time` cố
    định** → timing đẩy tất định. `damping/friction` thấp để khớp không ì.

### 2.3. Khối `ros2_control` — giao diện phần cứng

```xml
<ros2_control name="GazeboSimSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <joint name="pusher_joint">
    <command_interface name="position">
      <param name="min">-${stroke}</param>
      <param name="max">0.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```
- Khai báo cho `ros2_control` biết khớp `pusher_joint` nhận **lệnh vị trí**
  (`command_interface position`) và báo về **trạng thái** vị trí + vận tốc.
- **`<plugin>gz_ros2_control/GazeboSimSystem</plugin>`**: "phần cứng" ở đây là mô
  phỏng Gazebo — plugin này nối `ros2_control` với khớp trong Gazebo.

### 2.4. Plugin gz_ros2_control

```xml
<gazebo>
  <plugin filename="gz_ros2_control-system"
          name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(arg controllers_config)</parameters>
  </plugin>
</gazebo>
```
- Plugin này **khởi động một `controller_manager` bên trong Gazebo**, và nạp file
  YAML controller qua `$(arg controllers_config)` (chính đối số xacro ở trên).
- Đây là mắt xích để các controller (`joint_state_broadcaster`,
  `pusher_position_controller`) tồn tại và điều khiển được khớp.

---

## 3. `launch/pusher.launch.py` — Spawn robot + nạp controller

### 3.1. Xử lý xacro và cái bẫy "robot_description"

```python
controllers_config = os.path.join(pkg_controller, 'config', 'pusher_controllers.yaml')
xacro_file = os.path.join(pkg_desc, 'urdf', 'pusher.urdf.xacro')
robot_description = xacro.process_file(
    xacro_file, mappings={'controllers_config': controllers_config}).toxml()
```
- **`xacro.process_file(...).toxml()`**: biên dịch xacro → chuỗi URDF hoàn chỉnh,
  **tiêm** đường dẫn YAML controller vào đối số `controllers_config`.
- ⭐ **Điểm mấu chốt (bug đã biết):** file `pusher_controllers.yaml` được để trong
  package **`sorting_robot_controller`**, KHÔNG phải package này. Lý do:
  `controller_manager` **làm hỏng** đối số `--params-file` nếu đường dẫn chứa chuỗi
  **`robot_description`**. Vì tên package này là `sorting_robot_description` (có chứa
  `robot_description`!), nên nếu để YAML ở đây sẽ dính lỗi
  *"Couldn't parse params file"*. (Ghi chú này đã lưu trong bộ nhớ dự án.)

### 3.2. Các node

```python
robot_state_publisher = Node(..., parameters=[{'robot_description': robot_description}])
spawn_pusher = Node(package='ros_gz_sim', executable='create',
    arguments=['-topic', 'robot_description', '-name', 'pusher'])
joint_state_broadcaster = Node(package='controller_manager', executable='spawner',
    arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'])
pusher_controller = Node(package='controller_manager', executable='spawner',
    arguments=['pusher_position_controller', '--controller-manager-timeout', '60'])
```
- **`robot_state_publisher`**: publish URDF lên topic `robot_description` và phát TF.
- **`spawn_pusher`** (`create`): đọc URDF từ topic `robot_description` và **spawn**
  robot vào Gazebo.
- **Hai `spawner` của controller_manager**: nạp và kích hoạt lần lượt
  `joint_state_broadcaster` (đọc trạng thái khớp) và `pusher_position_controller`
  (nhận lệnh vị trí). `--controller-manager-timeout 60` = chờ tối đa 60s cho manager sẵn sàng.

### 3.3. Điều phối bằng event handler (staggered)

```python
delayed_jsb = RegisterEventHandler(OnProcessExit(
    target_action=spawn_pusher,
    on_exit=[TimerAction(period=6.0, actions=[joint_state_broadcaster])]))
pusher_after_jsb = RegisterEventHandler(OnProcessExit(
    target_action=joint_state_broadcaster,
    on_exit=[pusher_controller]))
```
- **Vấn đề:** lệnh `create` **thoát ngay khi entity được xếp hàng**, nhưng plugin
  `gz_ros2_control` (và controller_manager, việc đọc YAML) chỉ hoàn tất **vài giây
  sau**. Nạp controller quá sớm → lỗi *"Failed loading controller"*.
- **`RegisterEventHandler(OnProcessExit(...))`**: đăng ký "khi tiến trình X kết thúc
  thì làm Y". Ở đây:
  1. Khi `spawn_pusher` xong → **chờ thêm 6s** rồi mới nạp `joint_state_broadcaster`
     (cho YAML kịp load).
  2. Khi `joint_state_broadcaster` nạp xong → mới nạp `pusher_position_controller`.
- Cách này **an toàn hơn** so với chỉ dùng `TimerAction` cứng, vì nó phản ứng theo
  sự kiện thực tế (tiến trình trước đã xong) thay vì đoán mò thời điểm.

### 3.4. `start_world`

```python
world_launch = IncludeLaunchDescription(..., condition=IfCondition(start_world))
```
- Mặc định `start_world:=true` → tự mở luôn Gazebo. Khi được `bringup` gọi với
  `start_world:=false`, world đã có sẵn nên khối này **bị bỏ qua** (tránh 2 Gazebo).

---

## 4. `setup.py` & `package.xml`

- `setup.py`: copy `urdf/` và `launch/*.launch.py` vào `share/`. `entry_points` trống
  (package không có node Python thực thi).

---

## 5. Lưu ý

- Nếu controller không nạp được: kiểm tra (1) đường dẫn YAML **không** chứa
  `robot_description`, (2) đủ thời gian chờ trước khi spawn controller.
- Cách điều khiển tay để test:
  ```bash
  ros2 topic pub -1 /pusher_position_controller/commands \
      std_msgs/Float64MultiArray "{data: [-0.6]}"   # duỗi
  ros2 topic pub -1 /pusher_position_controller/commands \
      std_msgs/Float64MultiArray "{data: [0.0]}"     # thu
  ```
