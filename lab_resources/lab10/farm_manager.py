#!/usr/bin/env python3
"""
=============================================================================
  FARM MANAGER — Student Exercise
  AgTech ROS Course | PHYS-4000 / AG-3000 | Lab 10
=============================================================================

  The "Master Brain" that switches between MANUAL and AUTO mode.
  Implements a Deadman's Switch for safety.

  Subscribes:
    /joy               (sensor_msgs/Joy)   — Joystick input
    /auto/cmd_vel      (geometry_msgs/Twist) — Autopilot commands

  Publishes:
    /cmd_vel           (geometry_msgs/Twist) — Final motor commands

  Usage:
    ros2 run lab10 farm_manager
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

MODE_MANUAL = 0
MODE_AUTO = 1


class FarmManager(Node):
    def __init__(self):
        super().__init__('farm_manager')

        # 1. INPUTS
        self.sub_joy = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.sub_auto = self.create_subscription(
            Twist, '/auto/cmd_vel', self.auto_callback, 10)

        # 2. OUTPUT (The physical hardware listens here)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.mode = MODE_MANUAL
        self.auto_cmd = Twist()
        self.get_logger().info(
            "Farm Manager Online. HOLD Deadman to drive. Press 'A' for Auto.")

    def auto_callback(self, msg):
        self.auto_cmd = msg

    def joy_callback(self, msg):
        final_cmd = Twist()

        # --- BUTTON MAPPING (adjust indices for your controller) ---
        btn_auto = msg.buttons[0]      # 'A' Button
        btn_deadman = msg.buttons[5]   # R1/RB Button

        # 1. STATE SWITCHING
        if btn_auto == 1:
            self.mode = MODE_AUTO
            self.get_logger().info("AUTOPILOT ENGAGED")

        # 2. EMERGENCY OVERRIDE
        if abs(msg.axes[1]) > 0.1 or btn_deadman == 1:
            if self.mode == MODE_AUTO:
                self.mode = MODE_MANUAL
                self.get_logger().warn("HUMAN TAKEOVER!")

        # 3. PRIORITY LOGIC
        if self.mode == MODE_MANUAL:
            if btn_deadman == 1:
                final_cmd.linear.x = 0.3 * msg.axes[1]
                final_cmd.angular.z = 1.0 * msg.axes[3]
            else:
                final_cmd.linear.x = 0.0
                final_cmd.angular.z = 0.0
        elif self.mode == MODE_AUTO:
            final_cmd = self.auto_cmd

        self.pub_cmd.publish(final_cmd)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FarmManager())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
