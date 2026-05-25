#!/usr/bin/env python3
"""
=============================================================================
  FARM MANAGER - Safety Arbiter (Keyboard Deadman)
  AgTech ROS Course | PHYS-4000 / AG-3000 | Lab 10
=============================================================================
  The keyboard's spacebar is the deadman switch. Hold it to enable motion,
  release it to stop. The autonomous controller (Mission Runner) publishes
  to /auto/cmd_vel, but those commands only reach /cmd_vel while BOTH:
    1. The spacebar is held, AND
    2. The /auto/cmd_vel stream is fresh (not stale).

  Subscribes:
    /auto/cmd_vel      (geometry_msgs/Twist) - Autopilot intent

  Publishes:
    /cmd_vel           (geometry_msgs/Twist) - Final motor commands

  SAFETY: if the spacebar is not held, /cmd_vel is zero. Period.
  SAFETY: if /auto/cmd_vel has gone stale, /cmd_vel is zero. Period.

  Usage:
    pip install pynput   (or pip install pynput --break-system-packages)
    ros2 run lab10 farm_manager
"""
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard


# --- ACKERMANN LIMITS ---
MAX_LINEAR  = 0.25   # m/s, forward speed cap (passthrough clamp)
MAX_ANGULAR = 1.0    # rad/s, yaw rate cap

# --- STALENESS LIMIT ---
# If we haven't heard from the Mission Runner in this long, treat the
# autonomous command as dead and publish zero. Half a second is plenty
# of slack at the Mission Runner's 20 Hz publish rate.
AUTO_CMD_TIMEOUT = 0.5   # seconds


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class FarmManager(Node):
    def __init__(self):
        super().__init__('farm_manager')

        self.create_subscription(
            Twist, '/auto/cmd_vel', self.on_auto_cmd, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Deadman state, written by the keyboard listener thread,
        # read by the ROS timer. A lock keeps the read atomic. ---
        self._lock = threading.Lock()
        self._deadman_held = False
        self._last_auto_cmd = Twist()
        self._last_auto_time = self.get_clock().now()

        # --- Keyboard listener runs in its own thread. pynput needs this
        # to capture keys without blocking the ROS event loop. ---
        self._listener = keyboard.Listener(
            on_press=self._on_key_press,
            on_release=self._on_key_release,
        )
        self._listener.daemon = True
        self._listener.start()

        # --- Fixed-rate publish. The deadman is checked here, NOT in the
        # keyboard callbacks. This guarantees the check runs even if no
        # keyboard event fires for a while. ---
        self.create_timer(0.05, self.publish_cmd)   # 20 Hz

        self.get_logger().info(
            "Farm Manager ONLINE. "
            "HOLD SPACEBAR to enable motion. "
            "Focus this terminal window."
        )

    # ----------------------------------------------------------------
    # Keyboard callbacks (run in the pynput thread, not the ROS thread)
    # ----------------------------------------------------------------
    def _on_key_press(self, key):
        if key == keyboard.Key.space:
            with self._lock:
                if not self._deadman_held:
                    self.get_logger().info("DEADMAN engaged (spacebar held).")
                self._deadman_held = True

    def _on_key_release(self, key):
        if key == keyboard.Key.space:
            with self._lock:
                if self._deadman_held:
                    self.get_logger().info("DEADMAN released. Motors stopped.")
                self._deadman_held = False

    # ----------------------------------------------------------------
    # ROS callbacks
    # ----------------------------------------------------------------
    def on_auto_cmd(self, msg: Twist):
        with self._lock:
            self._last_auto_cmd = msg
            self._last_auto_time = self.get_clock().now()

    def publish_cmd(self):
        cmd = Twist()

        # ============================================================
        # SAFETY LAYER. FIRST, ABSOLUTE, NON-BYPASSABLE.
        # ============================================================
        with self._lock:
            held = self._deadman_held
            auto = self._last_auto_cmd
            auto_time = self._last_auto_time

        # Gate 1: deadman must be held.
        if not held:
            self.pub_cmd.publish(cmd)   # zero
            return

        # Gate 2: auto command stream must be fresh. If the Mission
        # Runner crashes mid-mission, we don't want to keep replaying
        # its last command forever just because the student is still
        # holding the spacebar.
        age = (self.get_clock().now() - auto_time).nanoseconds / 1e9
        if age > AUTO_CMD_TIMEOUT:
            self.get_logger().warn(
                f"Auto command stale ({age:.1f}s). Holding zero.",
                throttle_duration_sec=2.0)
            self.pub_cmd.publish(cmd)   # zero
            return

        # Both gates passed. Pass through the autonomous command, clamped.
        cmd.linear.x  = clamp(auto.linear.x,  -MAX_LINEAR,  MAX_LINEAR)
        cmd.angular.z = clamp(auto.angular.z, -MAX_ANGULAR, MAX_ANGULAR)
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = FarmManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SAFETY: publish zero on shutdown. Always.
        stop = Twist()
        node.pub_cmd.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()