# Giải thích chi tiết module `sorting_robot_controller`

> Package **điều khiển cú đẩy**: chạy một **máy trạng thái (state machine)** biến
> lệnh `/reject_object` thành chuỗi động tác duỗi → giữ → thu của paddle. Package
> cũng **chứa file cấu hình controller** của `ros2_control` (đặt ở đây vì lý do
> kỹ thuật — xem mục 3).

---

## 1. Vai trò & cấu trúc

```
src/sorting_robot_controller/
├── sorting_robot_controller/pusher_controller_node.py   # State machine
├── config/pusher_controllers.yaml                       # Cấu hình ros2_control
├── launch/pusher_control.launch.py                      # Robot + node điều khiển
├── setup.py                                             # entry_point
└── package.xml
```

---

## 2. `pusher_controller_node.py` — Máy trạng thái đẩy cube

### 2.1. Các trạng thái

```python
class State(Enum):
    IDLE = 'IDLE'; WAIT = 'WAIT'; EXTEND = 'EXTEND'; HOLD = 'HOLD'; RETRACT = 'RETRACT'
```

Vòng đời một cú đẩy:
```
IDLE --nhận reject--> WAIT --> EXTEND --> HOLD --> RETRACT --> IDLE
```

### 2.2. Tham số & pub/sub

```python
self.declare_parameter('command_topic', '/pusher_position_controller/commands')
self.declare_parameter('extend_position', -0.6)   # vị trí duỗi (m, âm = ra băng chuyền)
self.declare_parameter('retract_position', 0.0)   # vị trí thu về
self.declare_parameter('trigger_delay', 1.5)      # chờ sau reject rồi mới duỗi
self.declare_parameter('extend_time', 0.6)        # thời gian duỗi
self.declare_parameter('hold_time', 0.8)          # thời gian giữ
self.declare_parameter('retract_time', 0.6)       # thời gian thu
```
- **`cmd_pub`** publish `std_msgs/Float64MultiArray` lên `command_topic` — đây là
  topic mà `forward_command_controller` (từ YAML) lắng nghe.
- **`reject_sub`** subscribe `/reject_object` (từ `decision_node`).
- **`joint_sub`** subscribe `/joint_states` **chỉ để log** vị trí khớp thực tế.
- Ngay lúc khởi tạo gọi `self._send(retract_position)` để paddle bắt đầu ở trạng thái thu.

### 2.3. `_send()` — gửi lệnh vị trí

```python
def _send(self, position):
    self.cmd_pub.publish(Float64MultiArray(data=[float(position)]))
```
- Đóng gói một giá trị vị trí thành mảng `[position]` (controller nhận list vì có thể
  điều khiển nhiều khớp; ở đây chỉ 1 khớp).

### 2.4. Cơ chế timer "một lần" — phần tinh tế nhất

```python
def _clear_timer(self):
    if self._timer is not None:
        self._timer.cancel()
        self.destroy_timer(self._timer)     # ⭐ phải DESTROY, không chỉ cancel
        self._timer = None

def _schedule(self, delay, action):
    self._clear_timer()
    def _once():
        self._clear_timer()
        action()
    self._timer = self.create_timer(delay, _once)
```
- Timer của `rclpy` là **tuần hoàn (periodic)** — không có timer "một lần" sẵn.
- **Vì sao phải `destroy_timer` chứ không chỉ `cancel`?** (xem comment trong code):
  `cancel()` chỉ ngừng timer bắn, nhưng nó **vẫn nằm trong danh sách** của executor,
  và executor quét mọi timer chết mỗi vòng spin. Mỗi cú đẩy tạo 4 timer → nếu không
  destroy, danh sách phình dần → **các khoảng delay bị trôi (drift) dài ra** càng chạy lâu.
- **`_schedule` mô phỏng one-shot**: hủy timer cũ, tạo timer mới mà callback `_once()`
  **tự hủy chính nó trước** khi chạy `action` → đảm bảo không bao giờ bắn lại lần 2.

### 2.5. Chuỗi trạng thái

```python
def on_reject(self, msg):
    if self.state is not State.IDLE:
        # đang bận xử lý cube trước -> bỏ qua (decision_node đã debounce)
        return
    self._pending = msg.class_name
    self.state = State.WAIT
    self._schedule(self.trigger_delay, self._do_extend)   # chờ cube tới paddle

def _do_extend(self):
    self.state = State.EXTEND; self._send(self.extend_position)
    self._schedule(self.extend_time, self._do_hold)

def _do_hold(self):
    self.state = State.HOLD
    self._schedule(self.hold_time, self._do_retract)

def _do_retract(self):
    self.state = State.RETRACT; self._send(self.retract_position)
    self._schedule(self.retract_time, self._do_idle)

def _do_idle(self):
    self.state = State.IDLE; self._pending = None
```
- **Bảo vệ trạng thái:** chỉ nhận reject mới khi đang `IDLE`. Nếu đang đẩy → bỏ qua
  (tránh chồng lệnh giữa chừng).
- **`trigger_delay`**: chờ để paddle **gặp cube đúng lúc cube tới**. Với ROI đặt
  ngay pusher, giá trị này rất nhỏ (0.2s trong `system_params.yaml`).
- Mỗi bước: đổi trạng thái → (có thể) gửi lệnh vị trí → hẹn bước kế tiếp sau N giây.
  Toàn bộ **thuần thời gian**, không phụ thuộc phản hồi khớp → vẫn chạy nếu
  `/joint_states` tạm mất.

---

## 3. `config/pusher_controllers.yaml` — Cấu hình ros2_control

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    pusher_position_controller:
      type: forward_command_controller/ForwardCommandController

pusher_position_controller:
  ros__parameters:
    joints: [pusher_joint]
    interface_name: position
```
- Khai báo hai controller:
  - **`joint_state_broadcaster`**: publish trạng thái khớp lên `/joint_states`.
  - **`pusher_position_controller`** kiểu **`ForwardCommandController`**: chuyển
    thẳng lệnh nhận trên `/pusher_position_controller/commands` thành lệnh vị trí
    cho `pusher_joint`. Đây chính là topic mà node ở mục 2 publish tới.
- **`update_rate: 100`**: controller_manager chạy 100 lần/giây.

> ⭐ **Vì sao YAML này ở đây, không ở `sorting_robot_description`?** (comment đầu file
> giải thích): `controller_manager` **làm hỏng** đối số `--params-file` nếu đường dẫn
> chứa chuỗi **`robot_description`**, gây lỗi *"Couldn't parse params file"*. Tên
> package `sorting_robot_description` có chứa chuỗi đó → phải để YAML ở package
> `sorting_robot_controller` (không chứa `robot_description`). Ghi chú này đã lưu
> trong bộ nhớ dự án.

---

## 4. `launch/pusher_control.launch.py`

```python
pusher_launch = IncludeLaunchDescription(.../pusher.launch.py,
    launch_arguments={'start_world': start_world}.items())
pusher_controller_node = Node(package='sorting_robot_controller',
    executable='pusher_controller_node', output='screen')
```
- **Tái sử dụng** `pusher.launch.py` của `sorting_robot_description` để dựng world +
  spawn robot + nạp controller, rồi **thêm** node điều khiển đẩy.
- Đây là cách test toàn bộ "phần cứng + điều khiển" độc lập, trước khi ghép vào `bringup`.

---

## 5. `setup.py`

```python
entry_points={'console_scripts': [
    'pusher_controller_node = sorting_robot_controller.pusher_controller_node:main'
]}
```
- Đăng ký node thực thi + copy `config/` và `launch/` vào `share/`.

---

## 6. Lưu ý tinh chỉnh

- **Cube không rơi khỏi băng chuyền?** Tăng `extend_position` sâu hơn (ví dụ -0.68
  thay vì -0.6) và/hoặc `hold_time` lâu hơn — xem lý do tính toán trong
  `system_params.yaml` và `bringup_module_explained.md`.
- **Đẩy sớm/muộn?** Chỉnh `trigger_delay` trước tiên.
