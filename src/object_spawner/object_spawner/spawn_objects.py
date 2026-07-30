"""Spawn các vật thể (box/cylinder) tại VỊ TRÍ CỐ ĐỊNH lên bàn trong Gazebo.

Demo đơn giản -> layout xác định (vị trí + kích thước + màu cố định) để Phase 7 hardcode
pose gắp dễ dàng và mỗi lần chạy giống hệt nhau.

Gọi service Gazebo `/world/<world>/create` (kiểu ros_gz_interfaces/srv/SpawnEntity),
service này phải được ros_gz_bridge cầu nối sang ROS trước (xem spawn_objects.launch.py).

Mỗi vật: tên obj_0..obj_{N-1}, kích thước gripper kẹp được, <inertial> + friction để không
trượt/xuyên bàn, nhúng plugin DetachableJoint (Phase 6).

Tham số (ros2 param):
  num_objects   (int,   default 5)     spawn N vật đầu trong FIXED_OBJECTS
  world         (string,default pick_world)
  table_top_z   (double,default 0.4)   cao độ mặt bàn (world frame)
"""
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SpawnEntity


# Layout CỐ ĐỊNH: cung bán kính 0.24 m trước mặt robot (base ở gốc), azimuth 50..130 độ,
# cách nhau ~8.4cm tâm-tới-tâm (khe ~5cm giữa 2 vật) -> gắp 1 vật KHÔNG đụng vật kế bên.
# Đã kiểm bằng harvest FK: cả 5 vật với tới được bằng grasp GẦN TOP-DOWN (nghiêng <11 độ),
# GRASP_CONFIGS/PLACE_CONFIG trong pick_and_place.py sinh cùng lượt (scratchpad harvest.py).
# Nếu đổi R/azimuth ở đây, phải sinh lại các config đó cho khớp.
# Mỗi entry: (kind, dims, x, y, rgba)
#   kind="box": dims=(sx,sy,sz)        kind="cyl": dims=(radius, length)
FIXED_OBJECTS = [
    ("box", (0.030, 0.030, 0.050),  0.1543, 0.1839, (0.85, 0.15, 0.15, 1.0)),  # đỏ   (az 50)
    ("cyl", (0.015, 0.050),         0.0821, 0.2255, (0.15, 0.70, 0.20, 1.0)),  # lục  (az 70)
    ("box", (0.028, 0.028, 0.045),  0.0000, 0.2400, (0.15, 0.35, 0.85, 1.0)),  # lam  (az 90)
    ("cyl", (0.016, 0.045),        -0.0821, 0.2255, (0.90, 0.75, 0.10, 1.0)),  # vàng (az 110)
    ("box", (0.030, 0.030, 0.050), -0.1543, 0.1839, (0.70, 0.20, 0.75, 1.0)),  # tím  (az 130)
]


def box_inertia(mass, sx, sy, sz):
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    return ixx, iyy, izz


def cyl_inertia(mass, r, h):
    ixx = iyy = mass * (3.0 * r * r + h * h) / 12.0
    izz = mass * r * r / 2.0
    return ixx, iyy, izz


def grasp_plugin_xml(name, robot_name, robot_link):
    """Plugin DetachableJoint đặt TRONG vật (Phase 6).

    Đặt plugin trong vật (không phải robot) để tránh bài toán con-gà-quả-trứng:
    robot đã tồn tại khi vật spawn nên child_model luôn tìm thấy.
    Fixed joint đóng băng pose tương đối lúc attach -> chọn joint6_flange (tâm lòng
    bàn tay, link sống sót sau khi URDF gộp fixed joint) làm mỏ neo ổn định.

    LƯU Ý: plugin gz-sim8 (8.14) KHÔNG hỗ trợ <suppress_initial_attach> -> vật LUÔN bị
    weld vào robot ngay lúc spawn (dính cứng vào cổ tay, tay chạy là vật chạy theo).
    -> grasp_manager gửi detach cho mọi vật lúc khởi động (detach_on_start) để "nhả" xuống bàn;
       khi gắp thì attach lại (plugin re-attach ở pose hiện tại). Điều khiển qua gz.msgs.Empty.
    """
    return f"""
      <plugin filename="gz-sim-detachable-joint-system"
              name="gz::sim::systems::DetachableJoint">
        <parent_link>link</parent_link>
        <child_model>{robot_name}</child_model>
        <child_link>{robot_link}</child_link>
        <attach_topic>/grasp/{name}/attach</attach_topic>
        <detach_topic>/grasp/{name}/detach</detach_topic>
        <output_topic>/grasp/{name}/state</output_topic>
      </plugin>"""


def make_box_sdf(name, sx, sy, sz, rgba, grasp_plugin=""):
    mass = 0.05
    ixx, iyy, izz = box_inertia(mass, sx, sy, sz)
    return _wrap_sdf(
        name, mass, ixx, iyy, izz,
        geometry=f"<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>",
        rgba=rgba, grasp_plugin=grasp_plugin,
    )


def make_cylinder_sdf(name, r, h, rgba, grasp_plugin=""):
    mass = 0.05
    ixx, iyy, izz = cyl_inertia(mass, r, h)
    return _wrap_sdf(
        name, mass, ixx, iyy, izz,
        geometry=f"<cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder>",
        rgba=rgba, grasp_plugin=grasp_plugin,
    )


def _wrap_sdf(name, mass, ixx, iyy, izz, geometry, rgba, grasp_plugin=""):
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
    </link>{grasp_plugin}
  </model>
</sdf>"""


class ObjectSpawner(Node):
    def __init__(self):
        super().__init__("object_spawner")
        self.declare_parameter("num_objects", len(FIXED_OBJECTS))
        self.declare_parameter("world", "pick_world")
        self.declare_parameter("table_top_z", 0.4)
        # Phase 6: nhúng plugin DetachableJoint vào mỗi vật để attach/detach lúc gắp.
        self.declare_parameter("enable_grasp_plugin", True)
        self.declare_parameter("robot_name", "mycobot_280")
        self.declare_parameter("attach_link", "joint6_flange")

        p = self.get_parameter
        self.num = min(p("num_objects").value, len(FIXED_OBJECTS))
        world = p("world").value
        self.table_z = p("table_top_z").value
        self.enable_grasp = p("enable_grasp_plugin").value
        self.robot_name = p("robot_name").value
        self.attach_link = p("attach_link").value

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

    def _build_object(self, name, spec):
        """Trả về (sdf, half_height) từ 1 entry FIXED_OBJECTS; half_height để đặt lên mặt bàn."""
        kind, dims, _x, _y, rgba = spec
        plugin = (grasp_plugin_xml(name, self.robot_name, self.attach_link)
                  if self.enable_grasp else "")
        if kind == "box":
            sx, sy, sz = dims
            return make_box_sdf(name, sx, sy, sz, rgba, plugin), sz / 2.0
        r, h = dims
        return make_cylinder_sdf(name, r, h, rgba, plugin), h / 2.0

    def spawn_all(self):
        ok = 0
        for i in range(self.num):
            name = f"obj_{i}"
            spec = FIXED_OBJECTS[i]
            x, y = spec[2], spec[3]
            sdf, half_h = self._build_object(name, spec)

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
