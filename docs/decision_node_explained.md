# Giải thích chi tiết module `decision_node`

> Node **ra quyết định**: nhận `DetectedObject` từ vision, áp chính sách màu
> (màu được chấp nhận → GIỮ, còn lại → LOẠI), và phát lệnh loại lên
> `/reject_object`. Có **cổng vị trí (position gate)** và **chống dội (debounce)**
> để mỗi cube chỉ tạo đúng một lệnh, đúng thời điểm.

---

## 1. Vai trò & cấu trúc

```
src/decision_node/
├── decision_node/decision_node.py   # Logic quyết định
├── launch/decision.launch.py        # Chạy riêng node (test)
├── setup.py                         # entry_point: decision_node
└── package.xml
```

**Luồng:** `/detected_object` → chính sách màu + cổng vị trí + debounce → `/reject_object`.

---

## 2. Bối cảnh thiết kế (đọc docstring đầu file)

Camera nhìn một dải băng chuyền, nên một cube bị phát hiện **từ trước** khi tới
pusher và **ở trong khung suốt cả quãng đi**. Nếu "bắn lệnh ngay khi thấy rồi chờ
một khoảng cố định" thì điểm mốc thời gian là tùy tiện → cú đẩy không khớp.

**Giải pháp:** chỉ bắn lệnh khi **tâm cube đã vượt qua một vạch pixel** đặt gần
phía pusher. Vì vạch này là một vị trí cố định trên băng chuyền, cú đẩy **tự hiệu
chỉnh** bất kể tốc độ băng chuyền.

> Lưu ý: trong cấu hình thực tế (`system_params.yaml`), `trigger_direction: none`
> — tức **tắt cổng vị trí** — vì `vision_node` đã dùng **ROI** để chỉ phát hiện
> cube đúng tại pusher. Cổng vị trí là cơ chế thay thế khi không dùng ROI.

---

## 3. `decision_node.py`

### 3.1. Tham số

```python
self.declare_parameter('accepted_color', 'blue')   # màu được GIỮ
self.declare_parameter('debounce_sec', 2.0)         # giãn cách tối thiểu giữa 2 lệnh
self.declare_parameter('trigger_axis', 'x')         # trục pixel theo hướng băng chuyền
self.declare_parameter('trigger_line_px', 460.0)    # vạch pixel kích hoạt
self.declare_parameter('trigger_direction', 'increasing')  # 'increasing'/'decreasing'/'none'
self.declare_parameter('debug_positions', False)    # log tâm pixel để hiệu chỉnh
```
> ⚠️ **Bẫy YAML:** khi đặt `trigger_axis` trong file YAML phải là `"y"` (có ngoặc
> kép), nếu không `y` bị YAML hiểu thành boolean `True`. Xem
> `bringup_module_explained.md` mục tham số.

### 3.2. `_past_trigger_line()` — cổng vị trí

```python
def _past_trigger_line(self, msg) -> bool:
    if self.trigger_direction == 'none':
        return True                                   # tắt cổng: bắn ngay khi thấy
    px = msg.position.x if self.trigger_axis == 'x' else msg.position.y
    if self.trigger_direction == 'decreasing':
        return px <= self.trigger_line_px
    return px >= self.trigger_line_px                 # 'increasing'
```
- Chọn trục pixel (`x` = cột, `y` = hàng) tùy hướng cube di chuyển trong ảnh.
- `increasing`: cube coi là "đã qua vạch" khi pixel **≥** vạch; `decreasing`: khi **≤** vạch.
- `none`: luôn trả `True` → không xét vị trí, quyết định ngay lần đầu thấy.

### 3.3. `on_detection()` — luồng quyết định

```python
if self.debug_positions:
    self.get_logger().info(..., throttle_duration_sec=0.3)   # log giới hạn tần suất

if msg.class_name == self.accepted_color:
    return                          # GIỮ: cho cube đi tiếp tới cuối băng chuyền

if not self._past_trigger_line(msg):
    return                          # chưa tới vạch kích hoạt -> chờ

now_ns = self.get_clock().now().nanoseconds
if self._last_reject_ns is not None:
    elapsed = (now_ns - self._last_reject_ns) / 1e9
    if elapsed < self.debounce_sec:
        return                      # cùng một cube còn trong khung -> bỏ qua

self._last_reject_ns = now_ns
self.pub.publish(msg)               # chuyển tiếp NGUYÊN message (kèm vị trí)
```

Giải thích các điểm quan trọng:
- **`throttle_duration_sec=0.3`**: tiện ích logger của ROS 2 — chỉ in log tối đa mỗi
  0.3s dù callback chạy nhiều lần, tránh spam terminal.
- **Chính sách màu:** đúng `accepted_color` → `return` (giữ). Khác → xét tiếp để loại.
- **Debounce theo thời gian:** một cube ở trong khung nhiều frame sẽ kích hoạt callback
  liên tục. So sánh thời điểm hiện tại với `_last_reject_ns`; nếu cách nhau **< `debounce_sec`**
  thì bỏ qua → **một cube chỉ sinh một lệnh loại**.
- **`self.pub.publish(msg)`**: chuyển tiếp **toàn bộ** `DetectedObject` (gồm cả vị trí)
  sang `/reject_object` để `pusher_controller_node` biết cube nào và ở đâu.
- Dùng **nanoseconds từ `get_clock()`** (đồng hồ ROS) thay vì `time.time()` để nhất
  quán với thời gian mô phỏng (sim time) nếu bật.

---

## 4. `launch/decision.launch.py`

Launch tối giản để test riêng, chỉ cho phép override `accepted_color`. Tham số đầy
đủ khi chạy hệ thống đến từ `bringup/system_params.yaml`.

---

## 5. Lưu ý

- `decision_node` **không** biết gì về servo/Gazebo — nó thuần logic, chỉ nói qua topic.
  Đây là ví dụ điển hình của kiến trúc **tách rời (decoupled)** trong ROS 2.
- Muốn đổi màu "đạt/lỗi": chỉ cần đổi `accepted_color`, không phải sửa code.
- Muốn cube đỏ **và** xanh lá đều bị loại (như demo): chỉ cần `accepted_color: blue`
  — mọi màu khác blue tự động bị loại.
