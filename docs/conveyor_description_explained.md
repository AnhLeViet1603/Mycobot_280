# Giải thích chi tiết module `conveyor_description`

> Package **giữ chỗ (placeholder/scaffold)** dành cho mô tả URDF/xacro của băng
> chuyền. Hiện tại băng chuyền được mô tả trực tiếp bằng **SDF** trong
> `conveyor_gazebo/worlds/conveyor.world`, nên package này **chưa chứa mô hình
> thực sự** — nhưng vẫn được tạo sẵn theo chuẩn để mở rộng sau này.

---

## 1. Vai trò & cấu trúc hiện tại

```
src/conveyor_description/
├── conveyor_description/__init__.py   # rỗng (đánh dấu Python package)
├── resource/conveyor_description       # file marker ament index (rỗng)
├── setup.py
├── setup.cfg
└── package.xml
```

> ⚠️ **Chưa có thư mục `urdf/`.** `setup.py` đã khai báo copy `urdf/*` nhưng thư mục
> đó chưa tồn tại — `glob('urdf/*')` chỉ trả về danh sách rỗng nên build vẫn chạy
> bình thường (không lỗi), chỉ là không copy gì.

---

## 2. `setup.py`

```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
],
...
entry_points={'console_scripts': []},   # không có node
```
- Cấu trúc y hệt các package ament_python khác: đăng ký ament index, copy
  `package.xml`, và (dự phòng) copy mọi file trong `urdf/`.
- `entry_points` trống — package chỉ để chứa tài nguyên mô tả, không có mã thực thi.

---

## 3. Vì sao package này tồn tại nhưng "trống"?

Trong ROS 2, có quy ước tách **mô tả robot (`*_description`)** khỏi phần Gazebo/điều
khiển, để tái sử dụng URDF cho RViz, TF, MoveIt... Package này được tạo sẵn theo quy
ước đó cho băng chuyền.

Tuy nhiên nhóm đã chọn mô tả băng chuyền **trực tiếp trong SDF world** (đơn giản hơn
cho một vật thể tĩnh + plugin C++), nên `conveyor_description` **chưa được dùng đến**.

---

## 4. Nếu muốn dùng trong tương lai

- Thêm thư mục `urdf/` với file `conveyor.urdf.xacro` mô tả băng chuyền.
- Khi đó `setup.py` đã sẵn sàng copy nó vào `share/conveyor_description/urdf/`.
- Có thể tách slab băng chuyền khỏi world để dùng lại (ví dụ nhiều băng chuyền,
  hoặc hiển thị trong RViz).

> Đối chiếu: `sorting_robot_description` là ví dụ **đã hoàn thiện** của một package
> `*_description` (có URDF thật + launch spawn) — xem
> `sorting_robot_description_explained.md`.
