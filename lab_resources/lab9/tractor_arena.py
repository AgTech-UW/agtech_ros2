#!/usr/bin/env python3
"""
=============================================================================
  TRACTOR ARENA — Arena-Compatible Controller
  AgTech ROS Course | PHYS-4000 / AG-3000 | Lab 9
=============================================================================

  Same P-controller as tractor_skeleton.py, but subscribes to /tractor/target
  so clicking in the Interactive Arena sets the goal dynamically.

  Subscribes:
    /tractor/gps       (turtlesim/Pose)  — Position from God Node or Arena
    /tractor/target    (turtlesim/Pose)  — Target from arena click
    /tractor/tracking  (std_msgs/Bool)   — Tag visibility status

  Publishes:
    /cmd_vel           (geometry_msgs/Twist) — Motor commands

  Usage:
    Terminal 1:  ros2 run lab9 interactive_arena
    Terminal 2:  ros2 run lab9 tractor_arena
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool
import math


class TractorArena(Node):
    def __init__(self):
        super().__init__('tractor_arena')

        # =====================================================================
        # PUBLISHERS & SUBSCRIBERS
        # =====================================================================
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_gps = self.create_subscription(
            Pose, '/tractor/gps', self.gps_callback, 10
        )
        self.sub_target = self.create_subscription(
            Pose, '/tractor/target', self.target_callback, 10
        )
        self.sub_tracking = self.create_subscription(
            Bool, '/tractor/tracking', self.tracking_callback, 10
        )

        # =====================================================================
        # CONTROLLER GAINS
        # =====================================================================
        self.Kp_dist = 0.3
        self.Kp_ang = 1.0

        # =====================================================================
        # LIMITS
        # =====================================================================
        self.MAX_LINEAR = 0.20
        self.MIN_LINEAR = 0.08
        self.MAX_ANGULAR = 0.50
        self.GOAL_TOLERANCE = 0.15

        # =====================================================================
        # STATE
        # =====================================================================
        self.pose = None
        self.is_tracking = False
        self.has_target = False
        self.goal_x = 0.0
        self.goal_y = 0.0

        # =====================================================================
        # CONTROL LOOP (20 Hz)
        # =====================================================================
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            'TRACTOR ARENA ONLINE | Waiting for target click...'
        )

    # =========================================================================
    # CALLBACKS
    # =========================================================================
    def gps_callback(self, msg):
        self.pose = msg

    def target_callback(self, msg):
        """Receive a new target from the interactive arena (mouse click)."""
        new_x, new_y = msg.x, msg.y
        # Only log on actual target change (arena re-publishes every tick)
        if not self.has_target or abs(new_x - self.goal_x) > 0.01 or abs(new_y - self.goal_y) > 0.01:
            self.goal_x = new_x
            self.goal_y = new_y
            self.has_target = True
            self.get_logger().info(
                f'NEW TARGET: ({self.goal_x:.2f}, {self.goal_y:.2f})'
            )

    def tracking_callback(self, msg):
        self.is_tracking = msg.data

    # =========================================================================
    # CONTROL LOOP
    # =========================================================================
    def control_loop(self):
        cmd = Twist()

        # No GPS yet
        if self.pose is None:
            self.pub_vel.publish(cmd)
            return

        # No target yet — wait for a click
        if not self.has_target:
            self.pub_vel.publish(cmd)
            return

        # Compute error
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Close enough — stop
        if distance < self.GOAL_TOLERANCE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_vel.publish(cmd)
            return

        # P-Controller
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.pose.theta

        # Normalize to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        # Linear velocity
        raw_linear = self.Kp_dist * distance
        cmd.linear.x = max(self.MIN_LINEAR, min(self.MAX_LINEAR, raw_linear))

        # Angular velocity
        raw_angular = self.Kp_ang * angle_error
        cmd.angular.z = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, raw_angular))

        # Slow down if pointing the wrong way
        if abs(angle_error) > math.pi / 2:
            cmd.linear.x = self.MIN_LINEAR

        self.pub_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TractorArena()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop = Twist()
        node.pub_vel.publish(stop)
        node.get_logger().info('Tractor Arena stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()