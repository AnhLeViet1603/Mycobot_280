"""pick_and_place — gắp lần lượt các vật (vị trí cố định) rồi thả vào khay (Phase 7).

Dùng moveit_py (tận dụng cấu hình MoveIt Phase 4) để plan CÁNH TAY tới các cấu hình khớp
precompute, GRIPPER điều khiển thẳng qua gripper_controller action (KHÔNG qua MoveIt — 2 khớp
ngón flop out-of-bounds làm MoveIt báo start-state invalid, plan fail), attach/detach qua
grasp_manager (Phase 6) — publish tên vật lên /grasp/attach|detach.

TRÁNH VA VẬT: nạp mọi vật vào MoveIt PLANNING SCENE làm collision object (chúng chỉ có trong
Gazebo, MoveIt vốn không biết -> nếu không nạp, planner vạch đường xuyên qua vật). Khi gắp vật
nào thì GỠ vật đó ra khỏi scene (để hạ xuống); vật còn lại vẫn là chướng ngại nên tay tự tránh
-> hết cảnh quét đổ vật kế / mang vật ra khay đụng vật khác.

Chuỗi cho mỗi vật obj_i (TIẾP CẬN THẲNG ĐỨNG cho tự nhiên):
  gỡ obj_i khỏi scene -> mở gripper -> PREGRASP (ngay trên vật) -> HẠ THẲNG xuống GRASP (đúng
  tâm vật) -> đóng gripper -> ATTACH -> NÂNG THẲNG lên PREGRASP -> ready (cao) -> pose thả ->
  mở gripper -> DETACH -> ready

Chạy qua launch (cần nạp param MoveIt): ros2 launch mycobot_moveit_config pick_and_place.launch.py
(yêu cầu bringup + spawn + grasp đang chạy).

Dùng JOINT-SPACE GOAL với cấu hình khớp PRECOMPUTE (GRASP_CONFIGS/PLACE_CONFIG) thay vì pose
goal: joint-space plan ổn định, KHÔNG phụ thuộc start-state (pose goal fail khi chained từ
'ready' trong Gazebo thật). Các config ứng layout cung r=0.24, azimuth 50..130 (object_spawner),
gắp GẦN TOP-DOWN, tư thế gắp đã né hàng xóm (sinh với vật kế làm obstacle). Bố cục thưa (~8.4cm).
Nếu đổi layout/kích thước, sinh lại config bằng tools/harvest.py -> harvest3.py -> harvest4.py
(harvest FK + snap tâm vật + né hàng xóm), rồi verify bằng verify_rt.py (scene-aware plan 5/5).
"""
import os
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.logging import get_logger
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

# Hình học vật để nạp vào MoveIt planning scene (giữ ĐỒNG BỘ với object_spawner). Import trực
# tiếp để 1 nguồn sự thật; nếu không import được thì bỏ qua scene (degrade về hành vi cũ).
try:
    from object_spawner.spawn_objects import FIXED_OBJECTS
except Exception:  # noqa: BLE001
    FIXED_OBJECTS = None
TABLE_TOP_Z = 0.40   # khớp object_spawner table_top_z

ARM_JOINTS = [
    "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
    "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
]

# Gripper KHÔNG dùng MoveIt (2 khớp không collision hay flop out-of-bounds -> MoveIt báo
# start-state invalid, plan fail). Gửi thẳng trajectory tới gripper_controller (như test_gripper).
GRIPPER_JOINTS = ["gripper_finger_joint", "gripper_right_joint"]
GRIPPER_OPEN = [0.0, 0.0]
GRIPPER_CLOSE = [-0.4, 0.4]

# GẮP THẲNG ĐỨNG (tránh cảnh quét ngang vào vật rồi mới attach -> trông giả). Mỗi vật có 2 pose:
#   PREGRASP: grasp_center ngay TRÊN vật (+6..8cm), gripper mở.
#   GRASP:    grasp_center TRÙNG ĐÚNG TÂM vật (lệch xy 0.0mm, z = mặt bàn + nửa cao vật) -> ngón
#             ôm đúng vật, attach ở đúng vị trí tiếp xúc.
# PREGRASP & GRASP CÙNG NHÁNH IK (chỉ vai/khuỷu/cổ-tay đổi, joint1/5/6 giữ nguyên) nên đi giữa
# 2 pose là HẠ/NÂNG THẲNG. Sinh bằng scratchpad harvest3.py (dùng grasp gần-top-down làm seed,
# IK snap vào tâm vật + IK pregrasp giữ đúng hướng, cùng nhánh). Full chain plan-able 5/5 (verify2).
# Sinh NÉ HÀNG XÓM: config sinh với các vật kế bên làm collision obstacle (scratchpad harvest4.py
# + solve3.py) nên tư thế gắp không chồm sang vật cạnh. obj_3 (kẹp giữa 2 vật) phải gắp THẲNG
# ĐỨNG (pitch=0) mới né được cả hai. Xác minh full-chain scene-aware plan-able 5/5 (verify_rt.py).
PREGRASP_CONFIGS = [
    [1.6982, -0.4546, -2.1149, -0.6258, -2.1035, 0.0118],   # obj_0 (+0.154,+0.184)
    [1.8556, -0.3545, -2.0992, -0.8316, -2.5114, 0.1173],   # obj_1 (+0.082,+0.226)
    [-1.4295, 0.8435, 1.5736, 0.5994, 0.8664, -0.0559],     # obj_2 (+0.000,+0.240)
    [-1.4001, 0.5568, 2.0431, 0.5334, 0.1707, -0.0014],     # obj_3 (-0.082,+0.226)
    [-1.7927, 0.8046, 1.3629, 1.6066, -1.2857, -0.6244],    # obj_4 (-0.154,+0.184)
]
GRASP_CONFIGS = [
    [1.6982, -1.2543, -1.9338, -0.0072, -2.1035, 0.0118],   # obj_0  z=0.425
    [1.8555, -1.1474, -2.0086, -0.1293, -2.5115, 0.1173],   # obj_1  z=0.425
    [-1.4295, 1.4761, 1.2790, 0.2613, 0.8664, -0.0559],     # obj_2  z=0.423
    [-1.4001, 1.3194, 1.8127, 0.0012, 0.1707, -0.0014],     # obj_3  z=0.423 (thẳng đứng, né obj_2+obj_4)
    [-1.7927, 1.3533, 1.2261, 1.1948, -1.2857, -0.6245],    # obj_4  z=0.425
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
        self.psm = moveit.get_planning_scene_monitor()

    # ---- planning scene (vật cản) -----------------------------------------
    # Nạp các vật vào planning scene làm collision object -> MoveIt vạch đường TRÁNH vật khác
    # (chúng chỉ tồn tại trong Gazebo, MoveIt vốn không biết). Khi gắp vật nào thì GỠ vật đó ra
    # để hạ xuống gắp được; các vật còn lại vẫn là chướng ngại. Nhờ vậy tay không quét đổ vật kế.
    @staticmethod
    def _collision_object(i, add=True):
        kind, dims, x, y, _rgba = FIXED_OBJECTS[i]
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = f"obj_{i}"
        if not add:
            co.operation = CollisionObject.REMOVE
            return co
        pr = SolidPrimitive()
        if kind == "box":
            half_h = dims[2] / 2.0
            pr.type = SolidPrimitive.BOX
            pr.dimensions = [dims[0], dims[1], dims[2]]
        else:  # cyl: dims=(radius, length)
            half_h = dims[1] / 2.0
            pr.type = SolidPrimitive.CYLINDER
            pr.dimensions = [dims[1], dims[0]]   # SolidPrimitive.CYLINDER = [height, radius]
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = TABLE_TOP_Z + half_h
        pose.orientation.w = 1.0
        co.primitives = [pr]
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD
        return co

    def scene_add_all(self, num):
        """Nạp num vật vào scene trong 1 khối read_write (nhiều khối RW làm moveit_py dễ segfault)."""
        if FIXED_OBJECTS is None:
            self.logger.warn("Không import được FIXED_OBJECTS -> BỎ QUA planning scene (dễ va vật).")
            return
        with self.psm.read_write() as scene:
            for i in range(num):
                scene.apply_collision_object(self._collision_object(i, add=True))
            scene.current_state.update()
        self.logger.info(f"Planning scene: đã nạp {num} vật làm vật cản.")

    def scene_remove(self, i):
        if FIXED_OBJECTS is None:
            return
        with self.psm.read_write() as scene:
            scene.apply_collision_object(self._collision_object(i, add=False))
            scene.current_state.update()

    # ---- primitives -------------------------------------------------------
    def _set_start_sanitized(self, component):
        """Đặt start state = khớp ARM hiện tại + GRIPPER về giá trị HỢP LỆ (mặc định).

        2 ngón gripper không có collision -> flop RA NGOÀI giới hạn khớp khi tay vung (memory:
        ros YAML/gripper). set_start_state_to_current_state() nuốt luôn giá trị lệch đó, khiến
        PlanningResponseAdapter 'ValidateSolution' báo trạng thái INVALID -> INVALID_MOTION_PLAN
        (fail tất định ở vật thứ 2+ khi ngón đã trôi). Chỉ lấy khớp arm thật, gripper reset về
        trong-giới-hạn (không ảnh hưởng thực thi: arm_controller chỉ điều khiển khớp arm)."""
        with self.psm.read_only() as scene:
            cur = scene.current_state
            arm_pos = [cur.joint_positions[j] for j in ARM_JOINTS]
        rs = RobotState(self.robot_model)
        rs.set_to_default_values()          # gripper -> mặc định (trong giới hạn)
        rs.set_joint_group_positions("arm", arm_pos)
        rs.update()
        component.set_start_state(robot_state=rs)

    def _plan_exec(self, component, label):
        result = None
        for attempt in range(3):            # RRT + scene sync: retry vài lần cho chắc
            self._set_start_sanitized(component)
            result = component.plan()
            if result:
                break
            self.logger.warn(f"[{label}] plan fail (thử {attempt + 1}/3), thử lại...")
            time.sleep(0.3)
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

        self.scene_remove(i)   # gỡ vật đang gắp khỏi scene để hạ xuống được (vật khác vẫn là cản)
        if not self.gripper_to("open"):
            return False
        if not self.arm_to_config(PREGRASP_CONFIGS[i], f"{name}:pregrasp"):  # tới ngay TRÊN vật
            return False
        if not self.arm_to_config(GRASP_CONFIGS[i], f"{name}:grasp"):        # HẠ THẲNG xuống vật
            return False
        if not self.gripper_to("closed"):
            return False
        self.attach(name)
        if not self.arm_to_config(PREGRASP_CONFIGS[i], f"{name}:lift"):      # NÂNG THẲNG lên
            return False
        if not self.arm_to_named("ready"):          # thu về pose CAO trước khi sang khay ->
            return False                            # tránh quét ngang tầm thấp xô đổ vật còn lại
        if not self.arm_to_config(PLACE_CONFIG, f"{name}:place"):
            return False
        self.gripper_to("open")
        self.detach(name)
        self.arm_to_named("ready")                  # lùi ra
        return True

    def run(self, num):
        self.arm_to_named("ready")
        n = min(num, len(GRASP_CONFIGS))
        # Nạp tất cả vật vào planning scene -> mọi chuyển động của tay TỰ TRÁNH các vật còn lại
        # (thứ tự gắp không còn quan trọng; đường mang ra khay cũng vòng qua thay vì quét đổ).
        self.scene_add_all(n)
        done = 0
        for i in range(n):
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
