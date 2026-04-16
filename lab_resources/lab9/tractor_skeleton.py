#!/usr/bin/env python3
"""
=============================================================================
  TRACTOR SKELETON — Student P-Controller Exercise
  AgTech ROS Course | PHYS-4000 / AG-3000 | Lab 9
=============================================================================

  Complete the TODO blocks to build a P-controller that drives
  the robot from its current position to the goal at (0, 0).

  Subscribes:
    /tractor/gps       (turtlesim/Pose)  — Position from God Node

  Publishes:
    /cmd_vel           (geometry_msgs/Twist) — Motor commands

  Usage:
    ros2 run lab9 tractor_skeleton
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class HardwareTractor(Node):
    def __init__(self):
        super().__init__('tractor_skeleton')

        # 1. THE PUBLISHER: Sends math to your motor controller
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # 2. THE SUBSCRIBER: Listens to the God Node for our location
        self.sub_gps = self.create_subscription(
            Pose, '/tractor/gps', self.update_pose, 10
        )

        self.pose = None
        self.got_first_gps = False

        # THE GOAL: Drive to the center of the arena
        self.target_x = 0.0
        self.target_y = 0.0

        # TODO 1: Add your Kp tuning variables from Lab 7 here.
        # (Hint: Physical hardware usually needs lower gains than Turtlesim!)
        # self.Kp_dist = ???
        # self.Kp_ang = ???

        # Diagnostic logging counter (prints every 40 ticks = ~2 seconds)
        self.log_counter = 0
        self.LOG_INTERVAL = 40

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            'TRACTOR SKELETON ONLINE | Waiting for GPS on /tractor/gps...'
        )

    def update_pose(self, msg):
        self.pose = msg
        if not self.got_first_gps:
            self.got_first_gps = True
            self.get_logger().info(
                f'GPS LOCKED | Position: ({msg.x:.2f}, {msg.y:.2f}) '
                f'θ={math.degrees(msg.theta):.1f}° | '
                f'Goal: ({self.target_x}, {self.target_y})'
            )

    def normalize_angle(self, angle):
        """Fixes the 360-degree wrap-around problem."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        # Don't do math if we haven't seen the God Node yet
        if self.pose is None:
            return

        cmd = Twist()

        # ==========================================================
        # TODO 2: THE P-CONTROLLER AUTOSTEER LOGIC
        # ==========================================================
        # Step A: Calculate the X and Y errors.
        #   dx = ???
        #   dy = ???

        # Step B: Calculate the Euclidean distance to the target.
        #   distance = ???

        # Step C: Calculate the angle_to_goal (using math.atan2).
        #   angle_to_goal = ???

        # Step D: Calculate the angle_error and normalize it.
        #   angle_error = self.normalize_angle(???)

        # Step E: Write the P-Controller logic.
        #   cmd.linear.x = ???
        #   cmd.angular.z = ???
        #   (Make sure to cap your max speed so the robot doesn't
        #    fly off the table!)

        # Step F: Write an IF statement to stop the robot
        #         if the distance is < 0.15 meters.

        # ==========================================================
        # DIAGNOSTIC LOG — Shows your controller's math every 2s
        # (You can remove this once everything works)
        # ==========================================================
        self.log_counter += 1
        if self.log_counter >= self.LOG_INTERVAL:
            self.log_counter = 0
            self.get_logger().info(
                f'GPS: ({self.pose.x:+.2f}, {self.pose.y:+.2f}) '
                f'θ={math.degrees(self.pose.theta):+.1f}° | '
                f'cmd_vel: lin={cmd.linear.x:+.3f} ang={cmd.angular.z:+.3f}'
            )

        # ==========================================================
        # HARDWARE SAFETY CLAMP (DO NOT REMOVE)
        # ==========================================================
        if cmd.angular.z > 0.5:
            cmd.angular.z = 0.5
        elif cmd.angular.z < -0.5:
            cmd.angular.z = -0.5

        self.pub_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HardwareTractor())
    rclpy.shutdown()


if __name__ == '__main__':
    main()