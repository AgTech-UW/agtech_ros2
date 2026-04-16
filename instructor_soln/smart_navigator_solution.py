#!/usr/bin/env python3
"""
=============================================================================
  SMART NAVIGATOR — SOLVED (Instructor Answer Key)
  DO NOT DISTRIBUTE TO STUDENTS
  AgTech ROS Course | PHYS-4000 / AG-3000 | Lab 10
=============================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

IDLE = 'IDLE'
DRIVING = 'DRIVING'
APPROACH = 'APPROACH'
HEADLAND = 'HEADLAND'
REVERSE = 'REVERSE'


class SmartNavigator(Node):
    def __init__(self):
        super().__init__('smart_navigator')

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_gps = self.create_subscription(
            Pose, '/tractor/gps', self.update_pose, 10)

        self.pose = None
        self.got_first_gps = False

        # Zigzag field pattern
        self.rows = [
            [(0.5, 0.5), (0.5, -0.5)],
            [(0.8, -0.5), (0.8, 0.5)],
            [(1.1, 0.5), (1.1, -0.5)],
        ]

        self.current_row = 0
        self.current_wp = 0

        self.state = IDLE
        self.turn_phase = 0

        # TODO 1 — SOLVED: Tuning parameters
        self.Ld = 0.3               # Pure Pursuit look-ahead (m)
        self.cruise_speed = 0.08    # Full speed (m/s)
        self.approach_speed = 0.04  # Slow near row end
        self.reverse_speed = 0.04   # Backing up
        self.turn_speed = 0.03      # During headland
        self.Kp_ang = 1.0           # Angular P-gain
        self.max_angular = 0.5      # Safety clamp

        self.approach_radius = 0.30
        self.goal_tolerance = 0.12
        self.reverse_tolerance = 0.10

        # Track overshoot distance for headland
        self.headland_start_x = 0.0
        self.headland_start_y = 0.0

        self.log_counter = 0
        self.LOG_INTERVAL = 40

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('SMART NAVIGATOR (SOLVED) | Waiting for GPS...')

    # ==================================================================
    # HELPERS
    # ==================================================================

    def update_pose(self, msg):
        self.pose = msg
        if not self.got_first_gps:
            self.got_first_gps = True
            self.get_logger().info(
                f'GPS LOCKED | ({msg.x:.2f}, {msg.y:.2f}) '
                f'theta={math.degrees(msg.theta):.1f} deg')
            self.state = DRIVING

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def distance_to(self, x, y):
        return math.sqrt((x - self.pose.x)**2 + (y - self.pose.y)**2)

    def angle_to(self, x, y):
        return math.atan2(y - self.pose.y, x - self.pose.x)

    def get_current_waypoint(self):
        return self.rows[self.current_row][self.current_wp]

    def get_row_end(self):
        return self.rows[self.current_row][-1]

    def heading_of_next_row(self):
        if self.current_row + 1 >= len(self.rows):
            return self.pose.theta
        row = self.rows[self.current_row + 1]
        return math.atan2(
            row[-1][1] - row[0][1],
            row[-1][0] - row[0][0]
        )

    def advance_waypoint(self):
        self.current_wp += 1
        if self.current_wp >= len(self.rows[self.current_row]):
            return True
        return False

    def advance_to_next_row(self):
        self.current_row += 1
        self.current_wp = 0
        if self.current_row >= len(self.rows):
            return True
        return False

    def clamp_angular(self, angular):
        return max(-self.max_angular, min(self.max_angular, angular))

    # ==================================================================
    # PURE PURSUIT (used by DRIVING and APPROACH)
    # ==================================================================

    def pure_pursuit_cmd(self, target, speed):
        """Compute Pure Pursuit steering toward target at given speed."""
        cmd = Twist()

        angle_to_target = self.angle_to(target[0], target[1])
        angle_error = self.normalize_angle(angle_to_target - self.pose.theta)

        # Curvature-based steering
        curvature = 2.0 * math.sin(angle_error) / self.Ld

        cmd.linear.x = speed
        cmd.angular.z = self.clamp_angular(curvature * speed)

        # If target is behind us, slow way down
        if abs(angle_error) > math.pi / 2:
            cmd.linear.x = 0.02
            cmd.angular.z = self.clamp_angular(self.Kp_ang * angle_error)

        return cmd

    # ==================================================================
    # CONTROL LOOP
    # ==================================================================

    def control_loop(self):
        if self.pose is None:
            return

        cmd = Twist()

        # ==========================================================
        # IDLE
        # ==========================================================
        if self.state == IDLE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # ==========================================================
        # DRIVING — TODO 2 SOLVED: Pure Pursuit
        # ==========================================================
        elif self.state == DRIVING:
            target = self.get_current_waypoint()
            dist = self.distance_to(target[0], target[1])

            if dist < self.approach_radius:
                self.state = APPROACH
                self.get_logger().info(
                    f'APPROACH | Near ({target[0]:.2f}, {target[1]:.2f})')
            else:
                cmd = self.pure_pursuit_cmd(target, self.cruise_speed)

        # ==========================================================
        # APPROACH — TODO 3 SOLVED: Slow down near row end
        # ==========================================================
        elif self.state == APPROACH:
            target = self.get_current_waypoint()
            dist = self.distance_to(target[0], target[1])

            if dist < self.goal_tolerance:
                row_done = self.advance_waypoint()
                if row_done:
                    if self.current_row + 1 >= len(self.rows):
                        self.state = IDLE
                        self.get_logger().info('ALL ROWS COMPLETE! IDLE.')
                    else:
                        self.state = HEADLAND
                        self.turn_phase = 0
                        self.headland_start_x = self.pose.x
                        self.headland_start_y = self.pose.y
                        self.get_logger().info('HEADLAND | Starting U-turn')
                else:
                    self.state = DRIVING
                    self.get_logger().info(
                        f'DRIVING | Next WP in row {self.current_row}')
            else:
                # Same as driving but slower
                cmd = self.pure_pursuit_cmd(target, self.approach_speed)

        # ==========================================================
        # HEADLAND — TODO 4 SOLVED: 3-phase U-turn
        # ==========================================================
        elif self.state == HEADLAND:
            next_heading = self.heading_of_next_row()
            heading_error = self.normalize_angle(
                next_heading - self.pose.theta)

            if self.turn_phase == 0:
                # Phase 0: Overshoot — drive straight past the row end
                cmd.linear.x = self.turn_speed
                cmd.angular.z = 0.0

                overshoot_dist = math.sqrt(
                    (self.pose.x - self.headland_start_x)**2 +
                    (self.pose.y - self.headland_start_y)**2)

                if overshoot_dist > 0.25:
                    self.turn_phase = 1
                    self.get_logger().info('HEADLAND Phase 1 | Turning hard')

            elif self.turn_phase == 1:
                # Phase 1: Turn hard toward next row heading
                cmd.linear.x = 0.02

                # Determine turn direction
                if heading_error > 0:
                    cmd.angular.z = self.max_angular
                else:
                    cmd.angular.z = -self.max_angular

                # Transition when roughly aligned
                if abs(heading_error) < math.radians(30):
                    self.turn_phase = 2
                    self.get_logger().info('HEADLAND Phase 2 | Straightening')

            elif self.turn_phase == 2:
                # Phase 2: Straighten out, gentle correction
                cmd.linear.x = self.turn_speed
                cmd.angular.z = self.clamp_angular(
                    0.5 * heading_error)

                if abs(heading_error) < math.radians(10):
                    # Done turning, advance to next row
                    all_done = self.advance_to_next_row()
                    if all_done:
                        self.state = IDLE
                        self.get_logger().info('ALL ROWS COMPLETE! IDLE.')
                    else:
                        # Go to REVERSE to back into row entry, or
                        # go straight to DRIVING if close enough
                        entry = self.rows[self.current_row][0]
                        if self.distance_to(entry[0], entry[1]) > 0.15:
                            self.state = REVERSE
                            self.get_logger().info(
                                f'REVERSE | Backing to row {self.current_row} entry')
                        else:
                            self.state = DRIVING
                            self.get_logger().info(
                                f'DRIVING | Row {self.current_row}')

        # ==========================================================
        # REVERSE — TODO 5 SOLVED: Back up with inverted steering
        # ==========================================================
        elif self.state == REVERSE:
            entry = self.rows[self.current_row][0]
            dist = self.distance_to(entry[0], entry[1])

            if dist < self.reverse_tolerance:
                self.state = DRIVING
                self.get_logger().info(
                    f'DRIVING | Row {self.current_row} aligned')
            else:
                # Angle to goal, offset by pi (going backward)
                angle_to_goal = self.angle_to(entry[0], entry[1])
                angle_error = self.normalize_angle(
                    angle_to_goal - self.pose.theta + math.pi)

                cmd.linear.x = -self.reverse_speed
                cmd.angular.z = self.clamp_angular(
                    -self.Kp_ang * angle_error)

        # ==========================================================
        # DIAGNOSTIC LOG
        # ==========================================================
        self.log_counter += 1
        if self.log_counter >= self.LOG_INTERVAL:
            self.log_counter = 0
            if self.current_row < len(self.rows):
                wp = self.get_current_waypoint()
                wp_str = f'({wp[0]:.2f}, {wp[1]:.2f})'
            else:
                wp_str = '(done)'
            self.get_logger().info(
                f'[{self.state}] Row {self.current_row} WP {self.current_wp} | '
                f'Pos: ({self.pose.x:+.2f}, {self.pose.y:+.2f}) '
                f'theta={math.degrees(self.pose.theta):+.1f} deg | '
                f'Target: {wp_str} | '
                f'cmd: lin={cmd.linear.x:+.3f} ang={cmd.angular.z:+.3f}')

        # ==========================================================
        # SAFETY CLAMP & PUBLISH
        # ==========================================================
        cmd.angular.z = self.clamp_angular(cmd.angular.z)
        self.pub_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SmartNavigator())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
