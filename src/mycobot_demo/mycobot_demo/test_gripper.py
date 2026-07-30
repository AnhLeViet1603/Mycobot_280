"""Test gripper: đóng rồi mở qua action của gripper_controller.

gripper_finger_joint: lower=-0.74 (đóng), upper=0.15 (mở).

Chạy:  ros2 run mycobot_demo test_gripper
       ros2 run mycobot_demo test_gripper --ros-args -p action:=open
       ros2 run mycobot_demo test_gripper --ros-args -p action:=close
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 2 khớp ngón đối xứng: [gripper_finger_joint (trái), gripper_right_joint (phải)]
# Đóng: trái ÂM, phải DƯƠNG (mirror). Biên độ vừa phải để ngón KHÔNG chồng lên nhau.
# Nếu chạy thấy 2 ngón lồng/chéo nhau -> đảo dấu (đổi CLOSE thành [0.4, -0.4]).
GRIPPER_JOINTS = ["gripper_finger_joint", "gripper_right_joint"]
OPEN_POS = [0.0, 0.0]
CLOSE_POS = [-0.4, 0.4]


class GripperTester(Node):
    def __init__(self):
        super().__init__("gripper_tester")
        self.declare_parameter("action", "cycle")  # cycle | open | close
        self.client = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
        )

    def send(self, positions, secs=1.5):
        traj = JointTrajectory()
        traj.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal bị từ chối")
            return
        rclpy.spin_until_future_complete(self, handle.get_result_async())

    def run(self):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Không thấy /gripper_controller/follow_joint_trajectory")
            return
        mode = self.get_parameter("action").value
        if mode == "open":
            self.get_logger().info("Mở gripper"); self.send(OPEN_POS)
        elif mode == "close":
            self.get_logger().info("Đóng gripper"); self.send(CLOSE_POS)
        else:
            self.get_logger().info("Đóng gripper"); self.send(CLOSE_POS)
            self.get_logger().info("Mở gripper"); self.send(OPEN_POS)
        self.get_logger().info("Xong.")


def main():
    rclpy.init()
    node = GripperTester()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
