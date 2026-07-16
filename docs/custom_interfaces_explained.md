# Giải thích chi tiết module `custom_interfaces`

> Package định nghĩa **message tùy chỉnh** dùng chung cho cả pipeline. Đây là
> "hợp đồng dữ liệu" giữa `vision_node`, `decision_node` và
> `pusher_controller_node`. Vì là interface package (sinh mã message), nó phải
> build bằng **`ament_cmake`** chứ không phải `ament_python`.

---

## 1. Vai trò

- Định nghĩa một message duy nhất: **`DetectedObject`** — mô tả một cube mà camera nhìn thấy.
- Được `rosidl` biên dịch thành class Python/C++ để các node khác `import`.

### Cấu trúc

```
src/custom_interfaces/
├── msg/
│   └── DetectedObject.msg   # Định nghĩa message
├── CMakeLists.txt           # Khai báo sinh mã interface
└── package.xml              # Metadata + dependency đặc thù interface
```

---

## 2. `msg/DetectedObject.msg`

```
# A single object detected by the vision node
string class_name              # "red" | "blue" | "green"
geometry_msgs/Point position   # approximate position (image or world frame)
```

### Giải thích

- **`string class_name`**: tên màu của cube — kết quả phân loại của `vision_node`.
- **`geometry_msgs/Point position`**: một điểm 3D (`x`, `y`, `z`) tái sử dụng từ
  package chuẩn `geometry_msgs`. Trong dự án này nó **không mang tọa độ thế giới**
  mà được "mượn" để chở dữ liệu pixel:
  - `position.x`, `position.y` = tọa độ **pixel** của tâm blob (centroid).
  - `position.z` = **diện tích** blob (dùng để ước lượng kích thước/khoảng cách).

  > Đây là một quy ước nội bộ (xem `vision_node.py` dòng 120–124). Việc tái dùng
  > `Point` thay vì tạo 3 field riêng giúp message gọn và tương thích công cụ sẵn có.

---

## 3. `CMakeLists.txt`

```cmake
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  DEPENDENCIES geometry_msgs
)
```

- **`rosidl_generate_interfaces(...)`**: dòng cốt lõi. Nó bảo hệ thống build sinh
  mã (Python/C++/...) cho các file `.msg` liệt kê.
- **`DEPENDENCIES geometry_msgs`**: **bắt buộc** vì message dùng kiểu
  `geometry_msgs/Point`. Thiếu dòng này → lỗi build "unknown type".
- **`rosidl_default_generators`**: bộ sinh mã interface, chỉ cần lúc build.

---

## 4. `package.xml` — vì sao khác các package khác

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>geometry_msgs</depend>

<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

- **`ament_cmake`** (không phải `ament_python`): các interface package **bắt buộc**
  dùng CMake vì `rosidl` sinh mã ở tầng C++.
- **`rosidl_default_generators`** (buildtool): công cụ sinh mã lúc build.
- **`rosidl_default_runtime`** (exec): thư viện cần lúc chạy để dùng message.
- **`<member_of_group>rosidl_interface_packages</member_of_group>`**: đánh dấu đây
  là package interface, để hệ thống sinh mã nhận diện đúng nhóm.

---

## 5. Cách các node sử dụng

```python
from custom_interfaces.msg import DetectedObject   # import class đã sinh mã
```

- `vision_node` **tạo và publish** `DetectedObject` lên `/detected_object`.
- `decision_node` **subscribe** và đọc `class_name` + `position` để quyết định.
- `pusher_controller_node` nhận lại `DetectedObject` chuyển tiếp trên `/reject_object`.

> ⚠️ Sau khi sửa file `.msg`, phải **build lại `custom_interfaces` trước**, rồi mới
> build các package phụ thuộc, nếu không chúng vẫn dùng bản message cũ.
