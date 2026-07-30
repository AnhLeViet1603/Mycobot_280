"""grasp_manager — quản lý attach/detach vật khi gắp (Phase 6).

Mỗi vật obj_i mang plugin DetachableJoint (nhúng lúc spawn ở object_spawner) lắng nghe
2 topic gz.msgs.Empty:  /grasp/obj_i/attach  và  /grasp/obj_i/detach.
ros_gz_bridge cầu 2 topic đó sang ROS (std_msgs/Empty). Node này cho pick node (Phase 7)
một giao diện gọn: publish tên vật (std_msgs/String) lên /grasp/attach hoặc /grasp/detach,
node sẽ bắn Empty tới topic tương ứng của vật đó và theo dõi trạng thái đã gắp.

Tham số:
  num_objects      (int, default 5)      số vật obj_0..obj_{n-1} để tạo sẵn publisher
  prefix           (string, default obj_)
  detach_on_start  (bool, default True)  gửi detach cho mọi vật lúc khởi động.
      CẦN THIẾT vì plugin DetachableJoint (gz-sim8 8.14) không có suppress_initial_attach
      -> vật bị weld vào cổ tay ngay lúc spawn; phải detach để "nhả" xuống bàn.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String


class GraspManager(Node):
    def __init__(self):
        super().__init__("grasp_manager")
        self.declare_parameter("num_objects", 5)
        self.declare_parameter("prefix", "obj_")
        self.declare_parameter("detach_on_start", True)
        n = self.get_parameter("num_objects").value
        prefix = self.get_parameter("prefix").value

        self.names = [f"{prefix}{i}" for i in range(n)]
        # Tạo sẵn publisher Empty cho từng vật để bridge kịp kết nối trước khi gắp.
        self.attach_pubs = {
            name: self.create_publisher(Empty, f"/grasp/{name}/attach", 1)
            for name in self.names
        }
        self.detach_pubs = {
            name: self.create_publisher(Empty, f"/grasp/{name}/detach", 1)
            for name in self.names
        }
        self.attached = set()

        self.create_subscription(String, "/grasp/attach", self._on_attach, 10)
        self.create_subscription(String, "/grasp/detach", self._on_detach, 10)
        self.get_logger().info(
            f"grasp_manager sẵn sàng, quản lý {n} vật: {', '.join(self.names)}"
        )

        # Nhả mọi vật khỏi cổ tay lúc khởi động. Gửi lặp vài lần vì bridge lazy +
        # discovery cần thời gian kết nối (message đầu dễ rớt trước khi khớp publisher).
        if self.get_parameter("detach_on_start").value:
            self._release_ticks = 5
            self._release_timer = self.create_timer(0.5, self._release_all_once)

    def _release_all_once(self):
        for name in self.names:
            self.detach_pubs[name].publish(Empty())
        self._release_ticks -= 1
        if self._release_ticks == 4:
            self.get_logger().info("detach_on_start: nhả tất cả vật xuống bàn...")
        if self._release_ticks <= 0:
            self._release_timer.cancel()
            self.attached.clear()

    def attach(self, name):
        pub = self.attach_pubs.get(name)
        if pub is None:
            self.get_logger().warn(f"attach: không biết vật '{name}' (num_objects đủ chưa?)")
            return
        pub.publish(Empty())
        self.attached.add(name)
        self.get_logger().info(f"ATTACH -> {name}")

    def detach(self, name):
        pub = self.detach_pubs.get(name)
        if pub is None:
            self.get_logger().warn(f"detach: không biết vật '{name}'")
            return
        pub.publish(Empty())
        self.attached.discard(name)
        self.get_logger().info(f"DETACH -> {name}")

    def _on_attach(self, msg):
        self.attach(msg.data)

    def _on_detach(self, msg):
        self.detach(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = GraspManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
