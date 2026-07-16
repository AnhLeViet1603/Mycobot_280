# Giải thích chi tiết module `vision_node`

> Node **thị giác máy tính**: nhận ảnh camera, dùng **OpenCV ngưỡng màu HSV** để
> tìm cube màu lớn nhất trong một vùng quan tâm (ROI), phân loại màu và publish
> `DetectedObject`. Đây là mắt của cả hệ thống.

---

## 1. Vai trò & cấu trúc

```
src/vision_node/
├── vision_node/vision_node.py   # Toàn bộ logic thị giác
├── setup.py                     # entry_point: vision_node
└── package.xml
```

**Luồng:** `/camera/image_raw` → phân tích màu → `/detected_object`
(+ tùy chọn `/vision/debug_image` để xem trực quan).

---

## 2. `vision_node.py`

### 2.1. Bảng ngưỡng màu HSV

```python
HSV_RANGES = {
    'red':   [((0, 100, 60), (10, 255, 255)),
              ((170, 100, 60), (179, 255, 255))],
    'green': [((40, 80, 40), (85, 255, 255))],
    'blue':  [((100, 100, 40), (130, 255, 255))],
}
```
- Mỗi màu = danh sách các khoảng `(lo, hi)` trong không gian **HSV** (Hue, Saturation, Value).
- **Vì sao đỏ có 2 khoảng?** Trong OpenCV, Hue chạy 0–179 và **màu đỏ nằm vắt qua
  mốc 0/180** (đỏ ở cả ~0 và ~179). Một khoảng không bao trọn được → cần 2 khoảng
  rồi hợp lại. Xanh lá/xanh dương liền mạch nên chỉ cần 1 khoảng.
- Dùng HSV thay vì RGB vì HSV **tách màu (Hue) khỏi độ sáng (Value)** → bền với thay đổi ánh sáng.

### 2.2. `__init__` — tham số & pub/sub

- Tham số: `image_topic`, `min_area` (lọc đốm nhỏ), `publish_debug`, và **ROI**
  (`roi_enabled`, `roi_x_min/max`, `roi_y_min/max`).
- **`CvBridge()`**: cầu nối chuyển đổi giữa `sensor_msgs/Image` (ROS) ↔ mảng numpy (OpenCV).
- Subscribe ảnh, publish `DetectedObject` lên `/detected_object`, và (nếu bật)
  publish ảnh debug lên `/vision/debug_image`.

### 2.3. `_roi_bounds()` — tính biên vùng quan tâm

```python
x1 = self.roi_x_max if self.roi_x_max >= 0 else w   # -1 nghĩa là "tới mép ảnh"
x0 = max(0, min(self.roi_x_min, w))
return x0, min(x1, w), y0, min(y1, h)
```
- Kẹp (clamp) các biên ROI vào trong kích thước ảnh `w×h`, tránh vượt biên.
- Quy ước: giá trị **-1** ở `max` nghĩa là "kéo tới mép ảnh".

### 2.4. `on_image()` — xử lý mỗi khung hình (phần lõi)

**Bước 1 — chuyển đổi ảnh:**
```python
frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
```
- Đổi message ROS → ảnh BGR (numpy) → sang không gian HSV để lọc màu.

**Bước 2 — tạo mask từng màu và giới hạn vào ROI:**
```python
for lo, hi in ranges:
    m = cv2.inRange(hsv, np.array(lo), np.array(hi))
    mask = m if mask is None else cv2.bitwise_or(mask, m)   # hợp 2 khoảng (đỏ)

if self.roi_enabled:
    mask[:y0, :] = 0; mask[y1:, :] = 0     # xóa ngoài ROI (trên/dưới)
    mask[:, :x0] = 0; mask[:, x1:] = 0     # xóa ngoài ROI (trái/phải)
```
- **`cv2.inRange`**: tạo ảnh nhị phân (trắng = pixel thuộc khoảng màu).
- **`cv2.bitwise_or`**: gộp 2 khoảng của màu đỏ thành một mask.
- **Ép mask ngoài ROI về 0**: chỉ giữ lại blob nằm trong ô ROI → cube chỉ được "thấy"
  đúng lúc đi ngang pusher (vì camera đặt ngay trên paddle).

**Bước 3 — tìm blob lớn nhất trong tất cả các màu:**
```python
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
for c in contours:
    area = cv2.contourArea(c)
    if area < self.min_area: continue          # bỏ đốm nhiễu nhỏ
    if best is None or area > best[0]:
        m = cv2.moments(c)
        if m['m00'] == 0: continue
        cx = m['m10'] / m['m00']; cy = m['m01'] / m['m00']   # tâm khối
        best = (area, color, cx, cy, c)
```
- **`findContours`**: tìm đường bao các vùng trắng trong mask.
- **`contourArea`**: diện tích; nhỏ hơn `min_area` bị loại (khử nhiễu).
- **`cv2.moments` → cx, cy**: tính **tâm khối (centroid)** của blob theo công thức
  moment: `cx = m10/m00`, `cy = m01/m00` (với `m00` = diện tích). `m00 == 0` bị bỏ
  qua để tránh chia cho 0.
- Vòng lặp giữ lại **blob có diện tích lớn nhất** trên mọi màu → cube nổi bật nhất.

**Bước 4 — publish kết quả:**
```python
out = DetectedObject()
out.class_name = color
out.position.x = float(cx)      # pixel x
out.position.y = float(cy)      # pixel y
out.position.z = float(best[0]) # diện tích blob
self.pub.publish(out)
```
- Đóng gói màu + tâm pixel + diện tích vào message (xem quy ước ở
  `custom_interfaces_explained.md`) và gửi cho `decision_node`.

**Bước 5 — ảnh debug (tùy chọn):**
- Vẽ contour vàng, chấm tâm đỏ, tên màu, và **khung ROI xanh dương**, rồi publish
  `/vision/debug_image` để xem bằng RViz/rqt — rất tiện khi tinh chỉnh ROI.

---

## 3. `main()` — vòng đời node

```python
rclpy.init(); node = VisionNode(); rclpy.spin(node)
... finally: node.destroy_node(); if rclpy.ok(): rclpy.shutdown()
```
- Mẫu chuẩn của node ROS 2: khởi tạo → `spin` (chạy callback tới khi Ctrl-C) → dọn
  dẹp. `if rclpy.ok()` tránh gọi `shutdown()` hai lần.

---

## 4. Phụ thuộc & lưu ý

- Cần `opencv-python`, `numpy`, `cv_bridge` — nếu thiếu `cv_bridge` thì `imgmsg_to_cv2` lỗi.
- Node **không nhận được ảnh** nếu bridge camera (trong `conveyor_gazebo`) chưa chạy
  hoặc camera không render (thiếu GPU/display).
- Ngưỡng HSV phụ thuộc màu vật liệu trong SDF; đổi màu cube trong `object_spawner`
  thì có thể phải chỉnh lại `HSV_RANGES`.
