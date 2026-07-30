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

LƯU Ý: myCobot 280 tầm với nhỏ -> top-down KHÔNG với tới được. Dùng JOINT-SPACE GOAL với cấu
hình khớp PRECOMPUTE bằng IK offline (GRASP_CONFIGS/PLACE_CONFIG) thay vì pose goal: joint-space
plan ổn định, KHÔNG phụ thuộc start-state (pose goal fail khi chained từ 'ready' trong Gazebo thật).
Các config ứng với layout cung r=0.18 (object_spawner) + hướng gắp rpy=(pi,1.4,atan2(y,x)) tại z=0.45.
Nếu đổi layout/kích thước, dò lại IK và cập nhật các mảng dưới (xem script scratchpad cfg.py).
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

# Cấu hình khớp GẮP cho từng vật (index khớp FIXED_OBJECTS). Precompute bằng IK offline,
# SEED TỪ 'ready' để nằm gần ready (nhánh nhất quán, plan ready->config dễ). Đã verify
# collision-free + plan ready->config = True cho cả 5 (scratchpad chain3.py).
GRASP_CONFIGS = [
    [1.9832, -1.8571, 0.293, -1.5792, -0.6108, -1.399],    # obj_0 (+0.095,+0.153)
    [2.2576, -1.8572, 0.2933, -1.5794, -0.6108, -1.399],   # obj_1 (+0.050,+0.173)
    [2.5392, -1.5825, -0.2977, -1.2631, -0.6104, -1.399],  # obj_2 (+0.000,+0.180)
    [-0.9412, 1.59, 0.2813, 1.2745, 1.911, -1.3961],       # obj_3 (-0.050,+0.173)
    [-0.6669, 1.5904, 0.2804, 1.275, 1.9109, -1.3961],     # obj_4 (-0.095,+0.153)
]
# Cấu hình khớp THẢ (khay đích ~(-0.15,0.06,0.47)).
PLACE_CONFIG = [-2.4733, -1.2225, -0.8745, -1.0463, -0.53, -1.3992]


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
