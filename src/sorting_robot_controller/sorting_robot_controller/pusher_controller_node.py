"""Drive the servo pusher in response to reject commands (Phase 7).

Subscribes to ``/reject_object`` (published by ``decision_node`` whenever a
non-accepted cube is seen) and runs a small state machine that extends the
prismatic pusher across the belt, holds it, then retracts:

    IDLE --reject--> WAIT --> EXTEND --> HOLD --> RETRACT --> IDLE

The pusher's ``forward_command_controller`` takes a position command on
``/pusher_position_controller/commands`` (``std_msgs/Float64MultiArray``):
``[extend_position]`` slides the paddle across the belt (toward -Y),
``[retract_position]`` (0.0) pulls it back clear of the belt.

Timing matters: the cube is detected by the camera some distance *before* it
reaches the pusher, so we wait ``trigger_delay`` seconds after the reject
before extending, so the paddle meets the cube as it arrives.

State is monitored against ``/joint_states`` only for logging; the sequence is
purely time-driven so it works even if joint feedback is briefly unavailable.
"""

from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from custom_interfaces.msg import DetectedObject


class State(Enum):
    IDLE = 'IDLE'
    WAIT = 'WAIT'
    EXTEND = 'EXTEND'
    HOLD = 'HOLD'
    RETRACT = 'RETRACT'


class PusherControllerNode(Node):
    def __init__(self):
        super().__init__('pusher_controller_node')

        # Command topic of the forward_command_controller loaded from
        # pusher_controllers.yaml.
        self.declare_parameter(
            'command_topic', '/pusher_position_controller/commands')
        # Extended (across the belt) and retracted (clear) joint positions [m].
        # Extend is negative because the joint slides toward -Y; keep it within
        # the URDF limit (stroke = 0.70).
        self.declare_parameter('extend_position', -0.6)
        self.declare_parameter('retract_position', 0.0)
        # Delay from receiving a reject to starting the extend, so the paddle
        # meets the cube as it arrives at the pusher [s].
        self.declare_parameter('trigger_delay', 1.5)
        # Time allowed for the paddle to travel out before we start holding [s].
        self.declare_parameter('extend_time', 0.6)
        # How long to keep the paddle extended to sweep the cube off [s].
        self.declare_parameter('hold_time', 0.8)
        # Time allowed for the paddle to travel back before returning to IDLE [s].
        self.declare_parameter('retract_time', 0.6)

        self.command_topic = self.get_parameter('command_topic').value
        self.extend_position = self.get_parameter('extend_position').value
        self.retract_position = self.get_parameter('retract_position').value
        self.trigger_delay = self.get_parameter('trigger_delay').value
        self.extend_time = self.get_parameter('extend_time').value
        self.hold_time = self.get_parameter('hold_time').value
        self.retract_time = self.get_parameter('retract_time').value

        self.state = State.IDLE
        self._timer = None
        self._pending = None  # class_name of the cube being handled

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, self.command_topic, 10)
        self.reject_sub = self.create_subscription(
            DetectedObject, '/reject_object', self.on_reject, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 10)

        self._joint_position = None

        # Make sure the pusher starts retracted.
        self._send(self.retract_position)

        self.get_logger().info(
            f'pusher_controller_node ready: cmd [{self.command_topic}], '
            f'extend {self.extend_position} m, trigger_delay '
            f'{self.trigger_delay} s')

    # --- I/O helpers -----------------------------------------------------

    def _send(self, position: float):
        self.cmd_pub.publish(Float64MultiArray(data=[float(position)]))

    def on_joint_state(self, msg: JointState):
        if 'pusher_joint' in msg.name:
            self._joint_position = msg.position[msg.name.index('pusher_joint')]

    # --- State machine ---------------------------------------------------

    def _schedule(self, delay: float, action):
        # One-shot timer: cancel any previous one first.
        if self._timer is not None:
            self._timer.cancel()
        self._timer = self.create_timer(delay, action)

    def on_reject(self, msg: DetectedObject):
        if self.state is not State.IDLE:
            # Busy sweeping a previous cube; ignore. debounce in decision_node
            # already collapses per-cube bursts.
            self.get_logger().warn(
                f'busy ({self.state.value}); ignoring reject '
                f'for {msg.class_name}')
            return

        self._pending = msg.class_name
        self.state = State.WAIT
        self.get_logger().info(
            f'reject {msg.class_name}: waiting {self.trigger_delay} s before '
            f'push')
        self._schedule(self.trigger_delay, self._do_extend)

    def _do_extend(self):
        self.state = State.EXTEND
        self._send(self.extend_position)
        self.get_logger().info(f'EXTEND to {self.extend_position} m')
        self._schedule(self.extend_time, self._do_hold)

    def _do_hold(self):
        self.state = State.HOLD
        self.get_logger().info(f'HOLD {self.hold_time} s')
        self._schedule(self.hold_time, self._do_retract)

    def _do_retract(self):
        self.state = State.RETRACT
        self._send(self.retract_position)
        self.get_logger().info(f'RETRACT to {self.retract_position} m')
        self._schedule(self.retract_time, self._do_idle)

    def _do_idle(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self.state = State.IDLE
        self.get_logger().info(f'IDLE (done sweeping {self._pending})')
        self._pending = None


def main(args=None):
    rclpy.init(args=args)
    node = PusherControllerNode()
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
