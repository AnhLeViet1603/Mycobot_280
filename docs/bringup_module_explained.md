# Giải thích chi tiết module `bringup`

> Tài liệu này giải thích vai trò của package `bringup` và ý nghĩa từng dòng code
> (đặc biệt các dòng "khó"). `bringup` là package **cấp cao nhất** (top-level):
> nó không chứa node xử lý riêng, mà đóng vai trò **nhạc trưởng** — gom toàn bộ
> hệ thống lại và khởi động mọi thứ bằng **một câu lệnh duy nhất**:
>
> ```bash
> ros2 launch bringup system.launch.py
> ```

---

## 1. Tổng quan vai trò

Package `bringup` là **điểm vào (entry point)** của cả demo phân loại cube trên
băng chuyền. Nó chịu trách nhiệm:

1. **Khởi động Gazebo** (thế giới băng chuyền + camera) thông qua package `conveyor_gazebo`.
2. **Bật các node xử lý ROS 2**: `vision_node`, `decision_node`, `pusher_controller_node`.
3. **Spawn robot pusher** và nạp controller `ros2_control` thông qua `sorting_robot_description`.
4. **Bật node spawn cube** (`object_spawner`).
5. **Điều phối thời điểm khởi động (staggered startup)** — thứ tự và độ trễ, để mọi thứ sẵn sàng đúng lúc.
6. **Tập trung tất cả tham số** vào một file YAML duy nhất để dễ tinh chỉnh.

### Cấu trúc thư mục

```
src/bringup/
├── bringup/
│   └── __init__.py            # File rỗng — đánh dấu đây là Python package
├── config/
│   └── system_params.yaml     # Tham số tập trung cho toàn bộ pipeline
├── launch/
│   └── system.launch.py       # Launch file chính — "nhạc trưởng"
├── resource/
│   └── bringup                # File marker cho ament resource index
├── package.xml                # Metadata + khai báo dependency của package
└── setup.py                   # Script cài đặt (build ament_python)
```

Đây là kiểu package **ament_python thuần túy** nhưng không có node — chỉ chứa
launch file và config. Vai trò của nó là *"glue"* (chất keo) nối các package khác.

---

## 2. `setup.py` — Script cài đặt package

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vanhdeptraibodoiqua',
    ...
    entry_points={
        'console_scripts': [
            # Trống — package này không có node thực thi
        ],
    },
)
```

### Giải thích các dòng quan trọng

- **`package_name = 'bringup'`**: tên package, dùng lại nhiều lần bên dưới để tránh gõ sai.

- **`packages=find_packages(exclude=['test'])`**: tự động tìm các thư mục Python
  (có `__init__.py`) để đóng gói, bỏ qua thư mục `test`.

- **`data_files`** — đây là phần *quan trọng nhất và hay gây khó hiểu*. Nó khai báo
  những file **không phải code Python** cần được **copy vào thư mục `install/`** khi
  build, để ROS 2 tìm thấy lúc chạy. Mỗi phần tử là một tuple `(đích, [danh_sách_file])`:

  | Dòng | Ý nghĩa |
  |------|---------|
  | `('share/ament_index/resource_index/packages', ['resource/bringup'])` | Đăng ký package vào **ament index** — cách ROS 2 biết package `bringup` tồn tại. File `resource/bringup` rỗng, chỉ cần *có mặt*. |
  | `('share/bringup', ['package.xml'])` | Copy `package.xml` vào thư mục share của package. |
  | `(os.path.join('share', package_name, 'launch'), glob(...'launch/*'))` | Copy **tất cả** file trong `launch/` vào `share/bringup/launch/`. |
  | `(os.path.join('share', package_name, 'config'), glob(...'config/*'))` | Tương tự cho `config/`. |

  > **`glob(os.path.join('launch', '*'))`**: `glob()` trả về danh sách mọi file khớp
  > mẫu `launch/*`. Nhờ vậy khi thêm launch file mới, không cần sửa `setup.py`.
  >
  > ⚠️ **Lý do bắt buộc phải có 2 dòng cuối**: nếu quên, `system.launch.py` và
  > `system_params.yaml` **sẽ không được copy** sang `install/`, và lệnh
  > `ros2 launch bringup system.launch.py` sẽ báo *"file not found"* dù file vẫn
  > nằm trong thư mục `src/`. Đây là lỗi kinh điển của người mới học ROS 2.

- **`entry_points={'console_scripts': []}`**: **để trống** — vì `bringup` không có
  node Python nào để chạy trực tiếp (`ros2 run`). Nó chỉ chạy qua `ros2 launch`.

---

## 3. `package.xml` — Metadata và dependency

```xml
<package format="3">
  <name>bringup</name>
  ...
  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>rviz2</exec_depend>
  ...
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Giải thích

- **`<buildtool_depend>ament_python</buildtool_depend>`**: package được build bằng
  công cụ `ament_python` (không phải `ament_cmake`). Điều này khớp với việc có `setup.py`.

- **`<exec_depend>` (dependency lúc chạy)**:
  - `ros_gz_sim`: cầu nối Gazebo ↔ ROS 2, cần cho việc chạy mô phỏng.
  - `rviz2`: công cụ trực quan hóa, dùng khi bật `rviz:=true`.

  > Lưu ý: đây chỉ liệt kê 2 dependency "trực tiếp" của launch file này. Các package
  > nội bộ (`vision_node`, `decision_node`...) không cần khai báo ở đây vì chúng được
  > gọi qua `ros2 launch` chứ không được import trực tiếp — tuy nhiên khai báo thêm
  > cũng là good practice để `rosdep` cài đủ.

- **`<test_depend>`**: các công cụ kiểm tra style code (copyright, flake8, pep257) —
  chỉ chạy khi test, không ảnh hưởng runtime.

- **`<build_type>ament_python</build_type>`**: xác nhận lại loại build cho `colcon`.

---

## 4. `config/system_params.yaml` — Tham số tập trung

Đây là nơi **duy nhất** để tinh chỉnh hành vi của toàn hệ thống mà không cần sửa code.
Cấu trúc YAML tuân theo chuẩn parameter của ROS 2:

```yaml
<tên_node>:
  ros__parameters:
    <tên_tham_số>: <giá_trị>
```

> **Điểm mấu chốt:** `<tên_node>` ở đầu mỗi khối **phải khớp chính xác** với tham số
> `name=` mà node đó chạy trong `system.launch.py`. Nếu tên không khớp, node sẽ
> **không nhận được tham số** và âm thầm dùng giá trị mặc định.

### Các khối tham số

**`object_spawner`** — điều khiển việc sinh cube:
```yaml
object_spawner:
  ros__parameters:
    world: conveyor_world      # Tên world trong Gazebo (phải khớp file .world)
    spawn_period: 4.0          # Chu kỳ sinh cube (giây)
    cube_size: 0.05            # Cạnh cube (m)
    spawn_x: -0.9              # Vị trí đầu băng chuyền
    spawn_z: 0.6               # Thả từ trên cao xuống
    jitter_y: 0.08             # Lệch ngẫu nhiên theo Y để trông tự nhiên
    color_weights: [1.0, 1.0, 1.0]  # Tỉ lệ đỏ / xanh_dương / xanh_lá
    cube_lifetime: 20.0        # Tự xóa cube sau 20s (dọn cuối băng chuyền)
```

**`vision_node`** — nhận diện màu:
```yaml
vision_node:
  ros__parameters:
    image_topic: /camera/image_raw   # Topic ảnh (từ ros_gz_bridge)
    min_area: 300.0                  # Bỏ qua đốm màu nhỏ hơn (lọc nhiễu)
    roi_enabled: true                # Chỉ nhận diện trong 1 ô nhỏ (ROI)
    roi_x_min: 220                   # Ô ROI đặt ngay dưới camera = ngay tại pusher
    roi_x_max: 420
    roi_y_min: 210
    roi_y_max: 270
```
> **Ý tưởng ROI (Region Of Interest):** thay vì quét cả ảnh, chỉ xét một ô chữ nhật
> nhỏ đặt đúng vị trí pusher. Nhờ vậy, cube chỉ được "nhìn thấy" đúng khoảnh khắc nó
> đi ngang paddle → thời điểm đẩy tự động chính xác mà không cần tính toán delay phức tạp.

**`decision_node`** — ra quyết định giữ/loại:
```yaml
decision_node:
  ros__parameters:
    accepted_color: blue     # Xanh dương -> GIỮ, còn lại -> LOẠI
    debounce_sec: 2.0        # Chống lặp: 1 cube chỉ tạo 1 lệnh loại
    trigger_axis: "y"        # ⚠ Phải để trong ngoặc kép!
    trigger_line_px: 200.0
    trigger_direction: none  # 'none' = bắn ngay khi thấy (ROI đã lo timing)
    debug_positions: true
```
> ⚠️ **Bẫy YAML kinh điển** (`trigger_axis: "y"`): trong YAML, các giá trị trần
> như `y`, `n`, `yes`, `no`, `on`, `off` bị hiểu thành **boolean** (`true`/`false`),
> không phải chuỗi. Node cần chuỗi `"y"` (trục Y), nên **bắt buộc đặt trong ngoặc kép**.
> Quên ngoặc kép → node nhận `True` thay vì `"y"` → lỗi khó lần.

**`pusher_controller_node`** — điều khiển piston đẩy:
```yaml
pusher_controller_node:
  ros__parameters:
    command_topic: /pusher_position_controller/commands
    extend_position: -0.68   # Vị trí duỗi hết (âm = trượt về phía -Y, ra băng chuyền)
    retract_position: 0.0    # Vị trí thu về (thoát khỏi băng chuyền)
    trigger_delay: 0.2       # Chờ 0.2s sau lệnh loại rồi mới đẩy
    extend_time: 0.7         # Thời gian paddle duỗi ra
    hold_time: 1.2           # Giữ đủ lâu để cube rơi khỏi mép
    retract_time: 0.7        # Thời gian thu paddle về
```
> **Vì sao `extend_position = -0.68` chứ không phải `-0.6`?** (comment trong file giải
> thích): mép -Y của băng chuyền ở `y = -0.25`, tâm paddle = `0.35 + vị_trí_joint`.
> Với `-0.6`, tâm cube chỉ tới `~ -0.29` (ngay mép, chưa rơi). Cần `~ -0.68` để đẩy
> tâm cube tới `~ -0.37`, hẳn ra khỏi băng chuyền. Đây là kết quả **tinh chỉnh thực nghiệm**.

---

## 5. `launch/system.launch.py` — Launch file chính (quan trọng nhất)

Đây là "bộ não" điều phối. File dùng **Python launch system** của ROS 2.

### 5.1. Import và cấu trúc hàm

```python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ...
    return LaunchDescription([ ... ])
```

- **`generate_launch_description()`**: hàm **bắt buộc** — ROS 2 gọi hàm này để lấy
  danh sách những thứ cần khởi động. Phải trả về một đối tượng `LaunchDescription`.
- Các thành phần chính:
  - `IncludeLaunchDescription`: **gọi lại một launch file khác** (tái sử dụng).
  - `Node`: khởi động một node ROS 2 cụ thể.
  - `TimerAction`: **hoãn** việc khởi động một nhóm action sau N giây.
  - `DeclareLaunchArgument` + `LaunchConfiguration`: khai báo và đọc tham số dòng lệnh.

### 5.2. Xác định đường dẫn package và file config

```python
pkg_bringup = get_package_share_directory('bringup')
pkg_conveyor = get_package_share_directory('conveyor_gazebo')
pkg_desc = get_package_share_directory('sorting_robot_description')

params = os.path.join(pkg_bringup, 'config', 'system_params.yaml')
```

- **`get_package_share_directory('bringup')`**: trả về đường dẫn tuyệt đối tới thư
  mục `share/bringup` trong `install/` (nơi `setup.py` đã copy file tới ở mục 2).
  Không hardcode đường dẫn → chạy được trên mọi máy.
- **`params`**: đường dẫn đầy đủ tới file YAML, dùng chung cho mọi node bên dưới.

### 5.3. Đọc tham số dòng lệnh

```python
rviz = LaunchConfiguration('rviz')
```

- **`LaunchConfiguration('rviz')`**: tạo một "biến trễ" (lazy) tham chiếu tới đối số
  `rviz`. Giá trị thực chỉ được xác định lúc chạy (khi người dùng gõ `rviz:=true`).
  Nó chưa phải giá trị thật ở đây, mà là một *placeholder* sẽ được thay thế sau.

### 5.4. Khai báo các thành phần (chưa chạy)

**(1) Thế giới Gazebo + camera bridge:**
```python
world = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_conveyor, 'launch', 'conveyor.launch.py')),
    launch_arguments={'rviz': rviz}.items(),
)
```
- **`IncludeLaunchDescription(...)`**: nhúng launch file `conveyor.launch.py` của
  package khác — tái sử dụng thay vì viết lại.
- **`PythonLaunchDescriptionSource(...)`**: cho biết launch file được nhúng là kiểu Python.
- **`launch_arguments={'rviz': rviz}.items()`**: **truyền tiếp** đối số `rviz` từ
  launch file này xuống launch file con. `.items()` chuyển dict thành danh sách tuple
  (định dạng mà API yêu cầu).

**(2) Các node xử lý:**
```python
vision = Node(
    package='vision_node', executable='vision_node',
    name='vision_node', output='screen', parameters=[params],
)
decision = Node( package='decision_node', executable='decision_node',
    name='decision_node', ... parameters=[params] )
pusher_controller = Node( package='sorting_robot_controller',
    executable='pusher_controller_node', name='pusher_controller_node',
    ... parameters=[params] )
```
- **`package` / `executable`**: package chứa node và tên file thực thi (khai báo trong `entry_points` của package đó).
- **`name='vision_node'`**: **tên node lúc chạy** — chính là tên phải khớp với khối trong `system_params.yaml` (mục 4).
- **`output='screen'`**: in log ra terminal (thay vì chỉ ghi file log).
- **`parameters=[params]`**: nạp file YAML tham số. ROS 2 tự lọc đúng khối theo tên node.

**(3) Robot pusher (phần cứng mô phỏng + ros2_control):**
```python
pusher = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_desc, 'launch', 'pusher.launch.py')),
    launch_arguments={'start_world': 'false'}.items(),
)
```
- **`start_world: 'false'`**: điểm tinh tế! Launch file `pusher.launch.py` bình thường
  tự khởi động Gazebo. Nhưng ở đây **world đã được khởi động** ở bước (1), nên phải
  báo nó **đừng** mở Gazebo lần nữa (tránh chạy 2 instance Gazebo xung đột).

**(4) Node spawn cube:**
```python
spawner = Node(
    package='object_spawner', executable='spawner_node',
    name='object_spawner', output='screen', parameters=[params],
)
```

### 5.5. Điều phối thời gian khởi động (phần "khó" nhất)

```python
return LaunchDescription([
    DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Open RViz with the camera image display.'),
    world,
    TimerAction(period=3.0, actions=[vision, decision, pusher_controller]),
    TimerAction(period=6.0, actions=[pusher]),
    TimerAction(period=15.0, actions=[spawner]),
])
```

Đây là **thứ tự và độ trễ khởi động (staggered startup)** — cực kỳ quan trọng vì
các thành phần phụ thuộc lẫn nhau và cần thời gian sẵn sàng:

| Thời điểm | Thành phần | Lý do trễ |
|-----------|-----------|-----------|
| `t = 0s` | `DeclareLaunchArgument('rviz')` | Khai báo đối số `rviz` (mặc định `false`). |
| `t = 0s` | `world` | Gazebo phải lên **đầu tiên** — mọi thứ khác dựa vào nó. |
| `t = 3s` | `vision`, `decision`, `pusher_controller` | Các node này chỉ *chờ topic*, nên lên sớm rồi nằm im chờ dữ liệu. |
| `t = 6s` | `pusher` | Chờ Gazebo ổn định vài giây rồi mới spawn robot vào world. |
| `t = 15s` | `spawner` | Lên **cuối cùng** — để pusher đã spawn xong và controller đã nạp trước khi cube đầu tiên xuất hiện. |

- **`TimerAction(period=N, actions=[...])`**: hoãn khởi động nhóm `actions` đúng `N`
  giây sau khi launch bắt đầu. Đây là cách "chắc ăn" (dù hơi thô) để xử lý phụ thuộc
  thời gian mà không cần cơ chế đồng bộ phức tạp.

  > **Vì sao cần staggered startup?** Nếu spawn cube ngay khi robot chưa sẵn sàng,
  > cube sẽ chạy qua pusher trước khi controller kịp nạp → không đẩy được. Nếu spawn
  > robot trước khi Gazebo lên → lỗi "world not found". Các mốc 3/6/15s là kết quả
  > tinh chỉnh để mọi thứ vào đúng vị trí trước khi có cube đầu tiên.

---

## 6. Tóm tắt luồng khởi động

```
ros2 launch bringup system.launch.py
        │
        ▼
[t=0s]  Gazebo world + camera bridge  ← conveyor_gazebo
        │
[t=3s]  vision_node ─ decision_node ─ pusher_controller_node  (chờ dữ liệu)
        │
[t=6s]  Spawn robot pusher + nạp ros2_control  ← sorting_robot_description
        │
[t=15s] object_spawner bắt đầu thả cube
        │
        ▼
   Pipeline chạy: cube → camera → vision → decision → pusher → đẩy cube lỗi
```

---

## 7. Những điểm dễ sai (checklist)

1. **Quên khai báo `launch/` và `config/` trong `data_files` của `setup.py`** → file
   không được copy sang `install/` → `ros2 launch` báo không tìm thấy file.
2. **Tên node trong launch (`name=`) không khớp khối trong YAML** → node không nhận tham số.
3. **Quên ngoặc kép cho giá trị `y`/`n`/`on`/`off` trong YAML** → bị hiểu thành boolean.
4. **Quên `start_world:='false'` khi include `pusher.launch.py`** → mở 2 Gazebo xung đột.
5. **Rút ngắn các mốc `TimerAction`** → thành phần chưa kịp sẵn sàng, cube bị bỏ lỡ.
6. **Đường dẫn param chứa chuỗi `robot_description`** → có thể gây lỗi spawn controller
   (xem ghi chú riêng của dự án về gz_ros2_control).
```

Sau khi chỉnh `setup.py` hoặc thêm file config, **luôn build lại**:

```bash
cd ~/ros2_ws
colcon build --packages-select bringup
source install/setup.bash
```
