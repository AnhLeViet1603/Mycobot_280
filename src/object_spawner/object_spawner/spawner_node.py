"""Periodically spawn random-colored cubes at the start of the conveyor.

Uses the Gazebo Sim ``create`` service directly through the gz-transport
Python bindings, so no ros_gz bridge is required. Colors match the demo
scenario (blue = keep, red/green = reject).
"""

import random

import rclpy
from rclpy.node import Node

from gz.transport13 import Node as GzNode
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean


# name -> RGBA diffuse color
COLORS = {
    'red': (0.8, 0.0, 0.0, 1.0),
    'blue': (0.0, 0.0, 0.8, 1.0),
    'green': (0.0, 0.8, 0.0, 1.0),
}


def cube_sdf(name: str, size: float, rgba) -> str:
    """Build an SDF string for a single colored cube with sane inertia."""
    r, g, b, a = rgba
    # Solid cube inertia: I = 1/6 * m * s^2 (about each axis).
    mass = 0.2
    inertia = (1.0 / 6.0) * mass * size * size
    return f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{inertia}</ixx><iyy>{inertia}</iyy><izz>{inertia}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <surface>
          <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class SpawnerNode(Node):
    def __init__(self):
        super().__init__('object_spawner')

        # Parameters.
        self.declare_parameter('world', 'conveyor_world')
        self.declare_parameter('spawn_period', 3.0)
        self.declare_parameter('cube_size', 0.05)
        # Spawn pose at the start of the belt (top surface ~0.55 m).
        self.declare_parameter('spawn_x', -0.9)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_z', 0.6)
        # Lateral jitter so cubes are not perfectly aligned.
        self.declare_parameter('jitter_y', 0.1)
        # Relative weights for red / blue / green.
        self.declare_parameter('color_weights', [1.0, 1.0, 1.0])

        self.world = self.get_parameter('world').value
        self.cube_size = self.get_parameter('cube_size').value
        period = self.get_parameter('spawn_period').value

        self.gz = GzNode()
        self.service = f'/world/{self.world}/create'
        self.count = 0

        self.timer = self.create_timer(period, self.spawn_cube)
        self.get_logger().info(
            f'object_spawner ready: service [{self.service}], '
            f'period {period}s')

    def spawn_cube(self):
        names = list(COLORS.keys())
        weights = self.get_parameter('color_weights').value
        color = random.choices(names, weights=weights, k=1)[0]

        self.count += 1
        model_name = f'cube_{color}_{self.count}'

        x = self.get_parameter('spawn_x').value
        y = self.get_parameter('spawn_y').value
        z = self.get_parameter('spawn_z').value
        jitter = self.get_parameter('jitter_y').value
        y += random.uniform(-jitter, jitter)

        req = EntityFactory()
        req.sdf = cube_sdf(model_name, self.cube_size, COLORS[color])
        req.name = model_name
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z
        req.allow_renaming = True

        ok, rep = self.gz.request(
            self.service, req, EntityFactory, Boolean, 1000)
        if ok and rep.data:
            self.get_logger().info(
                f'spawned {model_name} at '
                f'({x:.2f}, {y:.2f}, {z:.2f})')
        else:
            self.get_logger().warn(
                f'failed to spawn {model_name} (service {self.service})')


def main(args=None):
    rclpy.init(args=args)
    node = SpawnerNode()
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
