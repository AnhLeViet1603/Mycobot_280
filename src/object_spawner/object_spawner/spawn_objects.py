"""Sinh N vật thể (box/cylinder) kích thước ngẫu nhiên và spawn lên bàn trong Gazebo.

Gọi service Gazebo `/world/<world>/create` (kiểu ros_gz_interfaces/srv/SpawnEntity),
service này phải được ros_gz_bridge cầu nối sang ROS trước (xem spawn_objects.launch.py).

Mỗi vật:
  - loại random: box hoặc cylinder
  - kích thước random NHƯNG trong khoảng gripper kẹp được (width ~2-3.5cm)
  - màu random, tên duy nhất (obj_0..obj_{N-1})
  - đặt ngẫu nhiên trong vùng với tới của cánh tay, KHÔNG chồng lên nhau
  - có <inertial> + friction để không trượt/xuyên bàn

Tham số (ros2 param):
  num_objects   (int,   default 5)     số vật
  world         (string,default pick_world)
  table_top_z   (double,default 0.4)   cao độ mặt bàn (world frame)
  seed          (int,   default -1)    <0 = random thật; >=0 = tái lập được
  x_min,x_max   (double)               vùng spawn theo x (mặc định trước mặt robot)
  y_min,y_max   (double)
"""
import math
import random
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SpawnEntity


# Khoảng kích thước (m) — chọn để gripper myCobot kẹp được.
BOX_W_RANGE = (0.020, 0.035)     # cạnh đáy hình hộp
BOX_H_RANGE = (0.030, 0.055)     # chiều cao hộp
CYL_R_RANGE = (0.010, 0.018)     # bán kính trụ
CYL_H_RANGE = (0.030, 0.055)     # chiều cao trụ

MIN_SEP = 0.055                  # khoảng cách tâm tối thiểu giữa 2 vật (tránh chồng)
MAX_PLACE_TRIES = 100


def box_inertia(mass, sx, sy, sz):
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    return ixx, iyy, izz


def cyl_inertia(mass, r, h):
    ixx = iyy = mass * (3.0 * r * r + h * h) / 12.0
    izz = mass * r * r / 2.0
    return ixx, iyy, izz


def make_box_sdf(name, sx, sy, sz, rgba):
    mass = 0.05
    ixx, iyy, izz = box_inertia(mass, sx, sy, sz)
    return _wrap_sdf(
        name, mass, ixx, iyy, izz,
        geometry=f"<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>",
        rgba=rgba,
    )


def make_cylinder_sdf(name, r, h, rgba):
    mass = 0.05
    ixx, iyy, izz = cyl_inertia(mass, r, h)
    return _wrap_sdf(
        name, mass, ixx, iyy, izz,
        geometry=f"<cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder>",
        rgba=rgba,
    )


def _wrap_sdf(name, mass, ixx, iyy, izz, geometry, rgba):
    r, g, b, a = rgba
    return f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>{mass:.4f}</mass>
        <inertia>
          <ixx>{ixx:.6e}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy:.6e}</iyy><iyz>0</iyz><izz>{izz:.6e}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>{geometry}</geometry>
        <surface>
          <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>{geometry}</geometry>
        <material>
          <ambient>{r:.2f} {g:.2f} {b:.2f} {a:.2f}</ambient>
          <diffuse>{r:.2f} {g:.2f} {b:.2f} {a:.2f}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class ObjectSpawner(Node):
    def __init__(self):
        super().__init__("object_spawner")
        self.declare_parameter("num_objects", 5)
        self.declare_parameter("world", "pick_world")
        self.declare_parameter("table_top_z", 0.4)
        self.declare_parameter("seed", -1)
        self.declare_parameter("x_min", -0.12)
        self.declare_parameter("x_max", 0.12)
        self.declare_parameter("y_min", 0.14)
        self.declare_parameter("y_max", 0.26)

        p = self.get_parameter
        self.num = p("num_objects").value
        world = p("world").value
        self.table_z = p("table_top_z").value
        self.xr = (p("x_min").value, p("x_max").value)
        self.yr = (p("y_min").value, p("y_max").value)

        seed = p("seed").value
        self.rng = random.Random(None if seed < 0 else seed)
        if seed >= 0:
            self.get_logger().info(f"Dùng seed cố định = {seed} (tái lập được)")

        self.srv_name = f"/world/{world}/create"
        self.cli = self.create_client(SpawnEntity, self.srv_name)

    def wait_service(self, timeout=15.0):
        self.get_logger().info(f"Chờ service {self.srv_name} ...")
        if not self.cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f"Không thấy {self.srv_name}. Đã chạy ros_gz_bridge cầu service chưa? "
                f"(spawn_objects.launch.py tự bật bridge này)"
            )
            return False
        return True

    def _random_object(self, name):
        """Trả về (sdf, half_height) cho 1 vật random; half_height để đặt lên mặt bàn."""
        rgba = (self.rng.random(), self.rng.random(), self.rng.random(), 1.0)
        if self.rng.random() < 0.5:
            sx = self.rng.uniform(*BOX_W_RANGE)
            sy = self.rng.uniform(*BOX_W_RANGE)
            sz = self.rng.uniform(*BOX_H_RANGE)
            return make_box_sdf(name, sx, sy, sz, rgba), sz / 2.0
        r = self.rng.uniform(*CYL_R_RANGE)
        h = self.rng.uniform(*CYL_H_RANGE)
        return make_cylinder_sdf(name, r, h, rgba), h / 2.0

    def _random_xy(self, placed):
        """Tìm vị trí (x,y) không chồng lên các vật đã đặt."""
        for _ in range(MAX_PLACE_TRIES):
            x = self.rng.uniform(*self.xr)
            y = self.rng.uniform(*self.yr)
            if all(math.hypot(x - px, y - py) >= MIN_SEP for px, py in placed):
                return x, y
        return None

    def spawn_all(self):
        placed = []
        ok = 0
        for i in range(self.num):
            name = f"obj_{i}"
            xy = self._random_xy(placed)
            if xy is None:
                self.get_logger().warn(
                    f"Không tìm được chỗ trống cho {name} sau {MAX_PLACE_TRIES} lần thử "
                    f"(vùng spawn quá nhỏ hoặc quá nhiều vật) — bỏ qua."
                )
                continue
            x, y = xy
            sdf, half_h = self._random_object(name)

            req = SpawnEntity.Request()
            req.entity_factory.name = name
            req.entity_factory.sdf = sdf
            req.entity_factory.relative_to = "world"
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = self.table_z + half_h + 0.002  # nhích lên tránh kẹt mặt bàn
            pose.orientation.w = 1.0
            req.entity_factory.pose = pose

            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            res = future.result()
            if res is not None and res.success:
                placed.append((x, y))
                ok += 1
                self.get_logger().info(f"Spawn {name} @ ({x:+.3f}, {y:+.3f})")
            else:
                self.get_logger().error(f"Spawn {name} THẤT BẠI (success=False/timeout).")
        self.get_logger().info(f"Hoàn tất: {ok}/{self.num} vật.")
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = ObjectSpawner()
    try:
        if node.wait_service():
            node.spawn_all()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
