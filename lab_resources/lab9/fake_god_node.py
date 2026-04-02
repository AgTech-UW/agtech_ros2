#!/usr/bin/env python3
"""
=============================================================================
  FAKE GOD NODE — Mock Ceiling Camera Simulator
  AgTech ROS Course | PHYS-4000 / AG-3000 | Lab 9
=============================================================================

  Publishes a simulated robot pose on /tractor/gps that slowly drifts
  around the arena. This lets students verify their P-controller is
  reacting to changing conditions — not just a frozen number.

  Modes (set with --ros-args -p mode:=<mode>):
    static   — Fixed pose at (1.0, 0.5), good for first sanity check
    drift    — Slow drift in a figure-8 pattern (default)
    circle   — Orbits around the arena center

  Usage:
    ros2 run lab9 fake_god_node
    ros2 run lab9 fake_god_node --ros-args -p mode:=static
    ros2 run lab9 fake_god_node --ros-args -p mode:=circle
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math
import time


class FakeGodNode(Node):
    def __init__(self):
        super().__init__('fake_god_node')

        self.declare_parameter('mode', 'drift')
        self.mode = self.get_parameter('mode').value

        self.gps_pub = self.create_publisher(Pose, '/tractor/gps', 10)
        self.timer = self.create_timer(0.1, self.broadcast_fake_gps)
        self.start_time = time.monotonic()

        mode_desc = {
            'static': 'Fixed at (1.0, 0.5) — sanity check mode',
            'drift':  'Figure-8 drift — watch your controller react',
            'circle': 'Circular orbit around arena center',
        }

        self.get_logger().info(
            f'FAKE GOD NODE ONLINE | Mode: {self.mode}\n'
            f'  {mode_desc.get(self.mode, "Unknown mode")}'
        )

    def broadcast_fake_gps(self):
        msg = Pose()
        t = time.monotonic() - self.start_time

        if self.mode == 'static':
            msg.x = 1.0
            msg.y = 0.5
            msg.theta = -0.2

        elif self.mode == 'circle':
            # Slow circle: radius 1.0m, centered at (2.0, 2.0), ~60s per lap
            msg.x = 2.0 + 1.0 * math.cos(t * 0.1)
            msg.y = 2.0 + 1.0 * math.sin(t * 0.1)
            msg.theta = t * 0.1 + math.pi / 2  # tangent to circle

        else:  # drift (default)
            # Figure-8: slow, covers a good chunk of the arena
            msg.x = 2.0 + 1.0 * math.sin(t * 0.08)
            msg.y = 2.0 + 0.8 * math.sin(t * 0.16)
            msg.theta = math.atan2(
                0.8 * 0.16 * math.cos(t * 0.16),
                1.0 * 0.08 * math.cos(t * 0.08)
            )

        # Normalize theta
        while msg.theta > math.pi:
            msg.theta -= 2.0 * math.pi
        while msg.theta < -math.pi:
            msg.theta += 2.0 * math.pi

        self.gps_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FakeGodNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()