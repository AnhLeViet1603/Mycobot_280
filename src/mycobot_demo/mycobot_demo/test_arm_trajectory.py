"""Test cánh tay: gửi vài pose qua action FollowJointTrajectory của arm_controller.

Dùng để kiểm tra Phase 3 (arm_controller nhận và thực thi trajectory).

Chạy:  ros2 run mycobot_demo test_arm
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

# Vài pose (rad) để cánh tay vẫy qua lại
POSES = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.6, -0.5, 0.4, 0.0, 0.5, 0.0],
    [-0.6, -0.8, 0.9, 0.0, 0.3, 0.0],
    [0.0, -0.3, 0.3, 0.0, 0.0, 0.0],
]


class ArmTester(Node):
    def __init__(self):
        super().__init__("arm_tester")
        self.client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )

    def run(self):
        self.get_logger().info("Chờ action server arm_controller...")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Không thấy /arm_controller/follow_joint_trajectory")
            return

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        t = 0.0
        for pose in POSES:
            t += 2.0
            pt = JointTrajectoryPoint()
            pt.positions = pose
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(f"Gửi {len(POSES)} pose...")
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal bị từ chối")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("Xong trajectory cánh tay.")


def main():
    rclpy.init()
    node = ArmTester()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
