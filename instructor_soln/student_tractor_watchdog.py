#!/usr/bin/env python3
"""
=============================================================================
  STUDENT TRACTOR — God Node Client with Watchdog Safety
  AgTech ROS Course | PHYS-4000 / AG-3000
=============================================================================

  This is the UPGRADED student tractor template that includes:
    1. P-Controller (from Lab 7/9)
    2. Watchdog Timer — if no GPS for 300ms, E-STOP
    3. /tractor/tracking subscriber for explicit tag-loss detection
    4. Ackermann-safe velocity clamping

  This version is meant to replace the basic tractor_student.py skeleton
  once students have proven their P-controller works with the fake god node.

  Subscribes:
    /tractor/gps       (turtlesim/Pose)  — Position from God Node
    /tractor/tracking   (std_msgs/Bool)   — Is the tag currently visible?

  Publishes:
    /cmd_vel            (geometry_msgs/Twist) — Motor commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool
import math
import time


class StudentTractor(Node):
    def __init__(self):
        super().__init__('tractor_student')

        # =====================================================================
        # PUBLISHERS & SUBSCRIBERS
        # =====================================================================
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_gps = self.create_subscription(
            Pose, '/tractor/gps', self.gps_callback, 10
        )
        self.sub_tracking = self.create_subscription(
            Bool, '/tractor/tracking', self.tracking_callback, 10
        )

        # =====================================================================
        # CONTROLLER GAINS — Tune these for your robot!
        # =====================================================================
        self.Kp_dist = 0.3      # Proportional gain for distance error
        self.Kp_ang = 1.0       # Proportional gain for heading error

        # =====================================================================
        # HARDWARE SAFETY LIMITS (Ackermann constraints)
        # =====================================================================
        self.MAX_LINEAR = 0.20   # m/s  — keep it slow for safety
        self.MIN_LINEAR = 0.08   # m/s  — must be > 0 to allow turning
        self.MAX_ANGULAR = 0.50  # rad/s — hardware limit for MentorPi
        self.GOAL_TOLERANCE = 0.15  # meters — "close enough" threshold

        # =====================================================================
        # WATCHDOG CONFIGURATION
        # =====================================================================
        self.WATCHDOG_TIMEOUT = 0.3  # seconds — E-STOP if no GPS for this long

        # =====================================================================
        # STATE
        # =====================================================================
        self.pose = None
        self.last_gps_time = None
        self.is_tracking = False
        self.is_estopped = False

        # GOAL — hardcoded for Lab 9, parameterized for final project
        self.goal_x = 0.0
        self.goal_y = 0.0

        # =====================================================================
        # CONTROL LOOP
        # =====================================================================
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(
            f'TRACTOR ONLINE | Goal: ({self.goal_x}, {self.goal_y}) | '
            f'Watchdog: {self.WATCHDOG_TIMEOUT}s'
        )

    # =========================================================================
    # CALLBACKS
    # =========================================================================
    def gps_callback(self, msg):
        """Receive the (x, y, theta) from the God Node."""
        self.pose = msg
        self.last_gps_time = time.monotonic()

        # If we were e-stopped due to tag loss, announce recovery
        if self.is_estopped:
            self.get_logger().info('GPS RECOVERED — resuming control.')
            self.is_estopped = False

    def tracking_callback(self, msg):
        """Receive explicit tracking status from God Node."""
        self.is_tracking = msg.data

    # =========================================================================
    # SAFETY: WATCHDOG CHECK
    # =========================================================================
    def check_watchdog(self):
        """
        Returns True if the GPS data is STALE (too old).
        This is a dead-man's switch — if the camera loses the tag,
        we stop immediately rather than driving blind.
        """
        if self.last_gps_time is None:
            return True  # Never received GPS — don't move

        elapsed = time.monotonic() - self.last_gps_time
        if elapsed > self.WATCHDOG_TIMEOUT:
            if not self.is_estopped:
                self.get_logger().warn(
                    f'WATCHDOG E-STOP | No GPS for {elapsed:.2f}s | '
                    f'Publishing zero velocity!'
                )
                self.is_estopped = True
            return True

        return False

    # =========================================================================
    # MAIN CONTROL LOOP
    # =========================================================================
    def control_loop(self):
        cmd = Twist()

        # --- SAFETY FIRST: Check watchdog ---
        if self.check_watchdog():
            # No valid GPS — full stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_vel.publish(cmd)
            return

        # --- No pose received yet ---
        if self.pose is None:
            self.pub_vel.publish(cmd)
            return

        # --- COMPUTE ERROR ---
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # --- CLOSE ENOUGH? STOP. ---
        if distance < self.GOAL_TOLERANCE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_vel.publish(cmd)
            return

        # --- P-CONTROLLER ---
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.pose.theta

        # Normalize angle to [-pi, pi] (the "wrap" fix from Lab 7)
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        # Linear velocity: proportional to distance, clamped
        raw_linear = self.Kp_dist * distance
        cmd.linear.x = max(self.MIN_LINEAR, min(self.MAX_LINEAR, raw_linear))

        # Angular velocity: proportional to heading error, clamped
        raw_angular = self.Kp_ang * angle_error
        cmd.angular.z = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, raw_angular))

        # --- ACKERMANN SAFETY ---
        # If the heading error is very large (> 90°), the robot is pointing
        # the wrong way. Slow down linear to prevent wild arcs.
        if abs(angle_error) > math.pi / 2:
            cmd.linear.x = self.MIN_LINEAR

        self.pub_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StudentTractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Send one final stop command on Ctrl+C
        stop = Twist()
        node.pub_vel.publish(stop)
        node.get_logger().info('Tractor stopped by operator.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
