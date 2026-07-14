"""Decide whether a detected cube should be kept or rejected.

Subscribes to ``/detected_object`` (published by the vision node) and applies
a simple color policy: the accepted color is kept, everything else triggers a
reject command on ``/reject_object``.

Because the vision node republishes a detection on every camera frame, the
same cube would otherwise generate a burst of reject commands. A time-based
debounce collapses that burst into a single command per cube.
"""

import rclpy
from rclpy.node import Node

from custom_interfaces.msg import DetectedObject


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # Color that is allowed to pass; anything else is rejected.
        self.declare_parameter('accepted_color', 'blue')
        # Minimum seconds between two reject commands, so a single cube that
        # stays in frame for many frames only triggers one reject.
        self.declare_parameter('debounce_sec', 2.0)

        self.accepted_color = self.get_parameter('accepted_color').value
        self.debounce_sec = self.get_parameter('debounce_sec').value

        self._last_reject_ns = None

        self.sub = self.create_subscription(
            DetectedObject, '/detected_object', self.on_detection, 10)
        self.pub = self.create_publisher(
            DetectedObject, '/reject_object', 10)

        self.get_logger().info(
            f'decision_node ready: accepted_color [{self.accepted_color}], '
            f'debounce {self.debounce_sec}s')

    def on_detection(self, msg: DetectedObject):
        if msg.class_name == self.accepted_color:
            return  # KEEP: let it pass to the end of the belt.

        now_ns = self.get_clock().now().nanoseconds
        if self._last_reject_ns is not None:
            elapsed = (now_ns - self._last_reject_ns) / 1e9
            if elapsed < self.debounce_sec:
                return  # Same cube still in frame: ignore.

        self._last_reject_ns = now_ns
        # Forward the full detection so the controller knows the position.
        self.pub.publish(msg)
        self.get_logger().info(
            f'REJECT {msg.class_name} at '
            f'({msg.position.x:.1f}, {msg.position.y:.1f})')


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
