# Giải thích chi tiết module `object_spawner`

> Node Python **sinh cube ngẫu nhiên** (đỏ/xanh dương/xanh lá) ở đầu băng chuyền
> theo chu kỳ, và **tự xóa** cube khi chúng tới cuối băng chuyền. Điểm đặc biệt:
> nó gọi **thẳng service của Gazebo** qua `gz-transport`, **không cần** ros_gz bridge.

---

## 1. Vai trò & cấu trúc

```
src/object_spawner/
├── object_spawner/spawner_node.py   # Toàn bộ logic
├── launch/spawner.launch.py         # Chạy riêng node (test)
├── setup.py                         # entry_point: spawner_node
└── package.xml
```

---

## 2. `spawner_node.py`

### 2.1. Import "lạ" — gz-transport thay vì ROS 2 service

```python
from gz.transport13 import Node as GzNode
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.boolean_pb2 import Boolean
```
- Đây **không phải** message ROS 2 mà là binding Python của **Gazebo Transport**
  (`gz.transport13`) và protobuf messages của Gazebo (`gz.msgs10`).
- Nhờ vậy node vừa là **ROS 2 node** (kế thừa `rclpy.Node`) vừa **nói chuyện trực
  tiếp** với Gazebo — không cần bắc cầu topic/service qua `ros_gz`.

### 2.2. `cube_sdf()` — sinh SDF của cube bằng chuỗi f-string

```python
def cube_sdf(name, size, rgba) -> str:
    mass = 0.2
    inertia = (1.0 / 6.0) * mass * size * size    # I = 1/6 * m * s^2
    return f"""<sdf ...>
      <model name="{name}"> ... <box><size>{size} {size} {size}</size></box> ...
      <diffuse>{r} {g} {b} {a}</diffuse> ... </sdf>"""
```
- Trả về một **chuỗi SDF hoàn chỉnh** mô tả 1 cube: khối lượng, moment quán tính,
  hình hộp, ma sát, và màu.
- **`inertia = 1/6 * m * s^2`**: công thức moment quán tính của khối lập phương đặc
  quanh mỗi trục — đặt đúng để cube rơi/lăn tự nhiên trong vật lý.

### 2.3. `__init__` — tham số và timer

```python
self.declare_parameter('world', 'conveyor_world')
self.declare_parameter('spawn_period', 3.0)
... spawn_x/y/z, jitter_y, color_weights, cube_lifetime ...

self.gz = GzNode()
self.service = f'/world/{self.world}/create'
self.remove_service = f'/world/{self.world}/remove'

self.timer = self.create_timer(period, self.spawn_cube)
if self.cube_lifetime > 0:
    self.despawn_timer = self.create_timer(1.0, self.despawn_expired)
```
- **`self.service = /world/<world>/create`**: tên service chuẩn của Gazebo để tạo
  entity. Vì thế `world` phải khớp tên world trong file `.world` (`conveyor_world`).
- Hai timer: một để **spawn** theo `spawn_period`, một chạy mỗi 1s để **dọn cube hết hạn**.

### 2.4. `spawn_cube()` — tạo một cube

```python
color = random.choices(names, weights=weights, k=1)[0]
model_name = f'cube_{color}_{self.count}'
y += random.uniform(-jitter, jitter)

req = EntityFactory()
req.sdf = cube_sdf(model_name, self.cube_size, COLORS[color])
req.name = model_name
req.pose.position.x/y/z = ...
req.allow_renaming = True

ok, rep = self.gz.request(self.service, req, EntityFactory, Boolean, 1000)
```
- **`random.choices(..., weights=..., k=1)`**: chọn màu ngẫu nhiên theo **trọng số**
  `color_weights` (đỏ/xanh dương/xanh lá). `[1,1,1]` = đều nhau.
- **`random.uniform(-jitter, jitter)`**: lệch ngẫu nhiên theo Y để cube không thẳng hàng, trông tự nhiên.
- **`allow_renaming = True`**: nếu trùng tên, Gazebo tự đổi tên thay vì báo lỗi.
- **`self.gz.request(service, req, EntityFactory, Boolean, 1000)`**: gọi service
  Gazebo kiểu **request/response** (blocking), timeout 1000ms. Trả về `(ok, rep)`:
  `ok` = gọi thành công, `rep.data` = kết quả boolean từ Gazebo.
- Nếu thành công, lưu `(model_name, thời_điểm_spawn)` vào `self.spawned` để sau này dọn.

### 2.5. `despawn_expired()` — dọn cube cũ

```python
now_ns = self.get_clock().now().nanoseconds
while self.spawned:
    name, spawn_ns = self.spawned[0]
    if (now_ns - spawn_ns) / 1e9 < self.cube_lifetime:
        break   # cube cũ nhất chưa hết hạn -> các cube sau cũng chưa
    self.spawned.pop(0)
    req = Entity(); req.name = name; req.type = Entity.MODEL
    self.gz.request(self.remove_service, req, Entity, Boolean, 1000)
```
- `self.spawned` được giữ **theo thứ tự spawn (cũ nhất đầu danh sách)**. Nhờ vậy chỉ
  cần xét phần tử đầu: nếu nó chưa hết hạn thì **dừng luôn** (`break`) — các cube sau
  chắc chắn còn mới hơn. Đây là tối ưu tránh duyệt cả list mỗi giây.
- Cube quá `cube_lifetime` giây → gọi service `/world/<world>/remove` để xóa khỏi Gazebo.

---

## 3. `launch/spawner.launch.py`

Launch tối giản để **test riêng** node, cho phép truyền `world` và `spawn_period`
qua dòng lệnh. Trong hệ thống đầy đủ, tham số thực tế đến từ `bringup/system_params.yaml`.

---

## 4. `setup.py` — entry point

```python
entry_points={'console_scripts': [
    'spawner_node = object_spawner.spawner_node:main'
]}
```
- Khác `bringup` (rỗng), package này khai báo **node thực thi**: `ros2 run
  object_spawner spawner_node` sẽ gọi hàm `main()` trong `spawner_node.py`.

---

## 5. Lưu ý

- Node **phụ thuộc binding `gz.transport13` / `gz.msgs10`** — cần cài đúng phiên bản
  Gazebo tương ứng, nếu không sẽ lỗi `import`.
- Nếu spawn báo "failed": thường do **sai tên world** (`world` không khớp `.world`)
  hoặc Gazebo chưa khởi động xong (vì thế `bringup` để spawner chạy **cuối cùng**, t=15s).
