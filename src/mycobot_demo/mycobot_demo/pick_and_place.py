"""pick_and_place — gắp lần lượt các vật (vị trí cố định) rồi thả vào khay (Phase 7).

Dùng moveit_py (tận dụng cấu hình MoveIt Phase 4) để plan CÁNH TAY tới các cấu hình khớp
precompute, GRIPPER điều khiển thẳng qua gripper_controller action (KHÔNG qua MoveIt — 2 khớp
ngón flop out-of-bounds làm MoveIt báo start-state invalid, plan fail), attach/detach qua
grasp_manager (Phase 6) — publish tên vật lên /grasp/attach|detach.

Chuỗi cho mỗi vật obj_i:
  mở gripper -> tới pose gắp (chĩa về vật) -> đóng gripper -> ATTACH
  -> ready (nhấc) -> tới pose thả -> mở gripper -> DETACH -> ready

Chạy qua launch (cần nạp param MoveIt): ros2 launch mycobot_moveit_config pick_and_place.launch.py
(yêu cầu bringup + spawn + grasp đang chạy).

Dùng JOINT-SPACE GOAL với cấu hình khớp PRECOMPUTE (GRASP_CONFIGS/PLACE_CONFIG) thay vì pose
goal: joint-space plan ổn định, KHÔNG phụ thuộc start-state (pose goal fail khi chained từ
'ready' trong Gazebo thật). Các config ứng layout cung r=0.24, azimuth 50..130 (object_spawner),
gắp GẦN TOP-DOWN (nghiêng <11 độ). Bố cục thưa (~8.4cm giữa các vật) nên gắp vật này không
đụng vật kế bên. Nếu đổi layout/kích thước, sinh lại config bằng scratchpad harvest.py (harvest
FK: chọn cấu hình khớp đưa grasp_center tới vật với hướng ít nghiêng nhất, verify plan-able).
"""
import os
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.logging import get_logger
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

ARM_JOINTS = [
    "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
    "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
]

# Gripper KHÔNG dùng MoveIt (2 khớp không collision hay flop out-of-bounds -> MoveIt báo
# start-state invalid, plan fail). Gửi thẳng trajectory tới gripper_controller (như test_gripper).
GRIPPER_JOINTS = ["gripper_finger_joint", "gripper_right_joint"]
GRIPPER_OPEN = [0.0, 0.0]
GRIPPER_CLOSE = [-0.4, 0.4]

# Cấu hình khớp GẮP cho từng vật (index khớp FIXED_OBJECTS). Sinh bằng "harvest FK": lấy mẫu
# cấu hình khớp hợp lệ, chọn cái đưa grasp_center TỚI vị trí vật với hướng GẦN TOP-DOWN
# (nghiêng <11 độ) — không ép công thức hướng cố định. Mỗi config đã verify collision-free +
# plan ready->config = True (scratchpad harvest.py). Ứng layout cung r=0.24, az 50..130.
GRASP_CONFIGS = [
    [1.5503, -0.9718, -2.1387, -0.0744, -2.2512, -0.0008],  # obj_0 (+0.154,+0.184) tilt 2.8
    [2.0171, -0.7109, -2.1363, -0.4596, -2.3518, 0.15],     # obj_1 (+0.082,+0.226) tilt 7.3
    [-1.3763, 1.2789, 1.6197, 0.1092, 0.9192, -0.0669],     # obj_2 (+0.000,+0.240) tilt 4.8
    [1.7338, -2.2824, 0.8249, -1.435, 1.9574, 0.1275],      # obj_3 (-0.082,+0.226) tilt 7.7
    [-1.8443, 1.1645, 1.5469, 1.1844, -1.3255, -0.7505],    # obj_4 (-0.154,+0.184) tilt 10.1
]
# Cấu hình khớp THẢ (khay đích ~(-0.20,0.10), gần top-down tilt 3.0).
PLACE_CONFIG = [-0.0714, 1.5998, 0.8123, 0.5351, 1.3757, -0.1476]


class PickAndPlace:
    def __init__(self, moveit: MoveItPy, node: Node):
        self.moveit = moveit
        self.node = node
        self.logger = get_logger("pick_and_place")
        self.robot_model = moveit.get_robot_model()
        self.arm = moveit.get_planning_component("arm")
        self.attach_pub = node.create_publisher(String, "/grasp/attach", 10)
        self.detach_pub = node.create_publisher(String, "/grasp/detach", 10)
        self.gripper_client = ActionClient(
            node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
        )

    # ---- primitives -------------------------------------------------------
    def _plan_exec(self, component, label):
        component.set_start_state_to_current_state()
        result = component.plan()
        if not result:
            self.logger.error(f"[{label}] PLAN THẤT BẠI")
            return False
        exec_result = self.moveit.execute(result.trajectory, controllers=[])
        ok = bool(exec_result)
        status = getattr(exec_result, "status", "?")
        self.logger.info(f"[{label}] plan OK, exec status={status} -> {'OK' if ok else 'EXEC FAIL'}")
        time.sleep(1.0)   # WSL RTF thấp: chờ arm tới đích + /joint_states cập nhật trước bước sau
        return ok

    def arm_to_named(self, name):
        self.arm.set_goal_state(configuration_name=name)
        return self._plan_exec(self.arm, f"arm->{name}")

    def gripper_to(self, name):
        """Đóng/mở gripper qua action gripper_controller (không qua MoveIt)."""
        positions = GRIPPER_CLOSE if name == "closed" else GRIPPER_OPEN
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.logger.error(f"[gripper->{name}] không thấy gripper_controller action")
            return False
        traj = JointTrajectory()
        traj.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=1, nanosec=0)
        traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.logger.error(f"[gripper->{name}] goal bị từ chối")
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res_fut, timeout_sec=5.0)
        self.logger.info(f"[gripper->{name}] OK")
        time.sleep(0.3)
        return True

    def arm_to_config(self, cfg, label):
        """Plan tới cấu hình khớp (joint-space) — ổn định, không phụ thuộc start-state."""
        rs = RobotState(self.robot_model)
        rs.set_joint_group_positions("arm", cfg)
        rs.update()
        self.arm.set_goal_state(robot_state=rs)
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
    def pick_one(self, i):
        name = f"obj_{i}"
        self.logger.info(f"=== {name} ===")

        if not self.gripper_to("open"):
            return False
        if not self.arm_to_config(GRASP_CONFIGS[i], f"{name}:grasp"):
            return False
        if not self.gripper_to("closed"):
            return False
        self.attach(name)
        if not self.arm_to_named("ready"):          # nhấc lên
            return False
        if not self.arm_to_config(PLACE_CONFIG, f"{name}:place"):
            return False
        self.gripper_to("open")
        self.detach(name)
        self.arm_to_named("ready")                  # lùi ra
        return True

    def run(self, num):
        self.arm_to_named("ready")
        done = 0
        for i in range(min(num, len(GRASP_CONFIGS))):
            try:
                if self.pick_one(i):
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
    num = node.declare_parameter("num_objects", len(GRASP_CONFIGS)).value
    moveit = MoveItPy(node_name="pick_and_place_moveit")
    PickAndPlace(moveit, node).run(num)
    # moveit_py hay segfault (SIGSEGV) khi hủy MoveItCpp lúc thoát. Việc đã xong ->
    # thoát cứng bằng os._exit để bỏ qua destructor lỗi, tránh exit code -11 gây nhiễu.
    node.get_logger().info("Xong. Thoát.")
    os._exit(0)


if __name__ == "__main__":
    main()
