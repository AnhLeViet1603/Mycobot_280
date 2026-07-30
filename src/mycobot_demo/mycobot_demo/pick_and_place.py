"""pick_and_place — gắp lần lượt các vật (vị trí cố định) rồi thả vào khay (Phase 7).

Dùng moveit_py (tận dụng cấu hình MoveIt Phase 4) để plan cánh tay tới pose gắp tính từ
bảng FIXED_OBJECTS (object_spawner), điều khiển gripper bằng named state open/closed (SRDF),
và attach/detach qua grasp_manager (Phase 6) — publish tên vật lên /grasp/attach|detach.

Chuỗi cho mỗi vật obj_i:
  ready -> mở gripper -> pre-grasp (trên vật) -> hạ xuống vật -> đóng gripper -> ATTACH
  -> nhấc lên -> tới trên khay -> mở gripper -> DETACH -> nhấc lên -> ready

Chạy qua launch (cần nạp param MoveIt): ros2 launch mycobot_moveit_config pick_and_place.launch.py
(yêu cầu bringup + spawn + grasp đang chạy).

LƯU Ý: myCobot 280 tầm với nhỏ + KDL IK -> vài pose có thể không giải/không tới được.
Các hằng số dưới (ORIENT_*, *_HEIGHT, PLACE_*) là điểm tinh chỉnh chính khi test thật.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler

from moveit.planning import MoveItPy

from object_spawner.spawn_objects import FIXED_OBJECTS

PLANNING_FRAME = "world"
EEF_LINK = "grasp_center"          # tip của group arm (SRDF)

TABLE_Z = 0.4                      # mặt bàn (world frame)
APPROACH_HEIGHT = 0.06             # pre-grasp cao hơn đỉnh vật (m)
GRASP_CLEARANCE = 0.005            # grasp_center cách đỉnh vật khi kẹp (m)
# Hướng gripper chúc thẳng xuống (top-down). rpy quanh trục world; tinh chỉnh khi test.
ORIENT_RPY = (3.14159, 0.0, 0.0)

# Khay đích (world frame) — bên trái robot.
PLACE_XY = (-0.20, 0.0)
PLACE_DROP_HEIGHT = 0.10           # thả từ độ cao này so với mặt bàn


def object_xyh(spec):
    """(x, y, height) của 1 entry FIXED_OBJECTS."""
    kind, dims, x, y, _rgba = spec
    h = dims[2] if kind == "box" else dims[1]
    return x, y, h


class PickAndPlace:
    def __init__(self, moveit: MoveItPy, node: Node):
        self.moveit = moveit
        self.node = node
        self.logger = get_logger("pick_and_place")
        self.arm = moveit.get_planning_component("arm")
        self.gripper = moveit.get_planning_component("gripper")
        self.attach_pub = node.create_publisher(String, "/grasp/attach", 10)
        self.detach_pub = node.create_publisher(String, "/grasp/detach", 10)
        self.qx, self.qy, self.qz, self.qw = quaternion_from_euler(*ORIENT_RPY)

    # ---- primitives -------------------------------------------------------
    def _plan_exec(self, component, label):
        component.set_start_state_to_current_state()
        result = component.plan()
        if not result:
            self.logger.error(f"[{label}] PLAN THẤT BẠI")
            return False
        self.moveit.execute(result.trajectory, controllers=[])
        self.logger.info(f"[{label}] OK")
        time.sleep(0.3)
        return True

    def arm_to_named(self, name):
        self.arm.set_goal_state(configuration_name=name)
        return self._plan_exec(self.arm, f"arm->{name}")

    def gripper_to(self, name):
        self.gripper.set_goal_state(configuration_name=name)
        return self._plan_exec(self.gripper, f"gripper->{name}")

    def arm_to_pose(self, x, y, z, label):
        ps = PoseStamped()
        ps.header.frame_id = PLANNING_FRAME
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.x = self.qx
        ps.pose.orientation.y = self.qy
        ps.pose.orientation.z = self.qz
        ps.pose.orientation.w = self.qw
        self.arm.set_goal_state(pose_stamped_msg=ps, pose_link=EEF_LINK)
        return self._plan_exec(self.arm, label)

    def attach(self, name):
        for _ in range(3):
            self.attach_pub.publish(String(data=name))
            time.sleep(0.1)
        self.logger.info(f"ATTACH {name}")
        time.sleep(0.3)

    def detach(self, name):
        for _ in range(3):
            self.detach_pub.publish(String(data=name))
            time.sleep(0.1)
        self.logger.info(f"DETACH {name}")
        time.sleep(0.3)

    # ---- 1 vật ------------------------------------------------------------
    def pick_one(self, i, spec):
        name = f"obj_{i}"
        x, y, h = object_xyh(spec)
        top = TABLE_Z + h                       # đỉnh vật
        z_pre = top + APPROACH_HEIGHT           # pre-grasp / retreat
        z_grasp = top + GRASP_CLEARANCE         # grasp_center chạm đỉnh vật
        z_place = TABLE_Z + PLACE_DROP_HEIGHT
        self.logger.info(f"=== {name} @ ({x:+.2f},{y:+.2f}) h={h:.3f} ===")

        if not self.gripper_to("open"):
            return False
        if not self.arm_to_pose(x, y, z_pre, f"{name}:pre-grasp"):
            return False
        if not self.arm_to_pose(x, y, z_grasp, f"{name}:grasp"):
            return False
        if not self.gripper_to("closed"):
            return False
        self.attach(name)
        if not self.arm_to_pose(x, y, z_pre, f"{name}:lift"):
            return False
        # tới trên khay rồi thả
        if not self.arm_to_pose(PLACE_XY[0], PLACE_XY[1], z_pre, f"{name}:above-tray"):
            return False
        if not self.arm_to_pose(PLACE_XY[0], PLACE_XY[1], z_place, f"{name}:lower-tray"):
            return False
        self.gripper_to("open")
        self.detach(name)
        self.arm_to_pose(PLACE_XY[0], PLACE_XY[1], z_pre, f"{name}:retreat")
        return True

    def run(self, num):
        self.arm_to_named("ready")
        done = 0
        for i in range(min(num, len(FIXED_OBJECTS))):
            try:
                if self.pick_one(i, FIXED_OBJECTS[i]):
                    done += 1
                else:
                    self.logger.warn(f"obj_{i}: bỏ qua (một bước fail), sang vật kế.")
                    self.arm_to_named("ready")
            except Exception as e:  # noqa: BLE001 - demo: không để 1 vật làm sập cả vòng
                self.logger.error(f"obj_{i}: lỗi {e!r}, sang vật kế.")
        self.arm_to_named("home")
        self.logger.info(f"HOÀN TẤT pick-and-place: {done}/{num} vật.")
        return done


def main(args=None):
    rclpy.init(args=args)
    node = Node("pick_and_place_helper")
    num = node.declare_parameter("num_objects", len(FIXED_OBJECTS)).value
    moveit = MoveItPy(node_name="pick_and_place_moveit")
    try:
        PickAndPlace(moveit, node).run(num)
    finally:
        moveit.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
