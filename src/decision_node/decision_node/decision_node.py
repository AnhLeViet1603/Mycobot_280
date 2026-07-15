"""Decide whether a detected cube should be kept or rejected.

Subscribes to ``/detected_object`` (published by the vision node) and applies
a simple color policy: the accepted color is kept, everything else is a reject.

The overhead camera sees a wide stretch of the belt, so a cube is detected
long *before* it reaches the pusher. Firing the reject on first sight and then
waiting a fixed delay does not work: the cube is in frame for the whole
crossing and the delay's reference point is arbitrary, so the push never lines
up with the cube arriving at the pusher.

Instead we gate on the cube's actual position: a reject is emitted only once
the blob centroid has travelled past a trigger line placed near the pusher
side of the image. Because that line is a fixed place on the belt, the push is
self-correcting regardless of belt speed — the controller then only needs a
small ``trigger_delay`` for the short remaining travel to the paddle.

A time-based debounce collapses the burst of frames for one cube into a single
command.
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

        # Position gate. Which pixel axis of the centroid follows belt travel
        # ('x' = image columns, 'y' = image rows) and the pixel line the cube
        # must be past, on the pusher side, before we fire. `trigger_direction`
        # says which side counts as "past": 'increasing' fires when the
        # centroid pixel >= the line, 'decreasing' when it is <= the line.
        # Tune by watching `ros2 topic echo /detected_object` as a cube crosses
        # and reading off the centroid value when it is level with the pusher.
        self.declare_parameter('trigger_axis', 'x')
        self.declare_parameter('trigger_line_px', 460.0)
        self.declare_parameter('trigger_direction', 'increasing')
        # Set 'increasing'/'decreasing' to gate on position; set to 'none' to
        # disable the gate entirely and fire on first sight (restores the old
        # behaviour — pushes early, but proves the pipeline still works).
        # Turn on debug_positions to log every detection's centroid so you can
        # read the real pixel range and set trigger_line_px / axis / direction.
        self.declare_parameter('debug_positions', False)

        self.accepted_color = self.get_parameter('accepted_color').value
        self.debounce_sec = self.get_parameter('debounce_sec').value
        self.trigger_axis = self.get_parameter('trigger_axis').value
        self.trigger_line_px = self.get_parameter('trigger_line_px').value
        self.trigger_direction = self.get_parameter('trigger_direction').value
        self.debug_positions = self.get_parameter('debug_positions').value

        self._last_reject_ns = None

        self.sub = self.create_subscription(
            DetectedObject, '/detected_object', self.on_detection, 10)
        self.pub = self.create_publisher(
            DetectedObject, '/reject_object', 10)

        self.get_logger().info(
            f'decision_node ready: accepted_color [{self.accepted_color}], '
            f'gate {self.trigger_axis}-px {self.trigger_direction} '
            f'{self.trigger_line_px}, debounce {self.debounce_sec}s')

    def _past_trigger_line(self, msg: DetectedObject) -> bool:
        if self.trigger_direction == 'none':
            return True  # Gate disabled: fire on first sight.
        px = msg.position.x if self.trigger_axis == 'x' else msg.position.y
        if self.trigger_direction == 'decreasing':
            return px <= self.trigger_line_px
        return px >= self.trigger_line_px

    def on_detection(self, msg: DetectedObject):
        if self.debug_positions:
            self.get_logger().info(
                f'det {msg.class_name} px=({msg.position.x:.0f}, '
                f'{msg.position.y:.0f}) area={msg.position.z:.0f}',
                throttle_duration_sec=0.3)

        if msg.class_name == self.accepted_color:
            return  # KEEP: let it pass to the end of the belt.

        # Only fire once the cube has reached the pusher-side trigger line.
        if not self._past_trigger_line(msg):
            return

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
