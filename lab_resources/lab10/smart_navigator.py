#!/usr/bin/env python3
"""
=============================================================================
  SMART NAVIGATOR — Student State Machine Exercise
  AgTech ROS Course | PHYS-4000 / AG-3000 | Lab 10
=============================================================================

  Upgrade your Lab 9 "point and shoot" P-controller into a state machine
  that can handle multi-waypoint paths, headland turns, and reversing.

  States:
    IDLE      — Waiting for waypoints. Publishes zero velocity.
    DRIVING   — Following current row using Pure Pursuit.
    APPROACH  — Near row endpoint. Slowing down.
    HEADLAND  — Executing a U-turn to enter next row.
    REVERSE   — Backing up to align with next row entry.

  Subscribes:
    /tractor/gps       (turtlesim/Pose)  — Position from God Node

  Publishes:
    /cmd_vel           (geometry_msgs/Twist) — Motor commands
    (Remap to /auto/cmd_vel when running with farm_manager)

  Usage:
    ros2 run lab10 smart_navigator
    ros2 run lab10 smart_navigator --ros-args -r /cmd_vel:=/auto/cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


# =====================================================================
# STATES
# =====================================================================
IDLE = 'IDLE'
DRIVING = 'DRIVING'
APPROACH = 'APPROACH'
HEADLAND = 'HEADLAND'
REVERSE = 'REVERSE'


class SmartNavigator(Node):
    def __init__(self):
        super().__init__('smart_navigator')

        # Publisher & Subscriber
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_gps = self.create_subscription(
            Pose, '/tractor/gps', self.update_pose, 10)

        self.pose = None
        self.got_first_gps = False

        # ==========================================================
        # WAYPOINTS — Define your zigzag field pattern here.
        # Each row is a list of (x, y) tuples.
        # The robot drives row 0 forward, then headland turns into
        # row 1 (which goes the opposite direction), etc.
        # ==========================================================
        self.rows = [
            [(0.5, 0.5), (0.5, -0.5)],     # Row 0: drive "south"
            [(0.8, -0.5), (0.8, 0.5)],      # Row 1: drive "north"
            [(1.1, 0.5), (1.1, -0.5)],      # Row 2: drive "south"
        ]

        self.current_row = 0
        self.current_wp = 0   # Index within current row

        # ==========================================================
        # STATE MACHINE
        # ==========================================================
        self.state = IDLE

        # Headland turn phase tracker (0, 1, 2)
        self.turn_phase = 0

        # ==========================================================
        # TUNING PARAMETERS
        # ==========================================================
        # TODO 1: Set your tuning parameters.
        # Pure Pursuit look-ahead distance (meters)
        # self.Ld = ???

        # Speeds (m/s)
        # self.cruise_speed = ???
        # self.approach_speed = ???
        # self.reverse_speed = ???
        # self.turn_speed = ???

        # Tolerances (meters)
        self.approach_radius = 0.30   # Switch to APPROACH when this close
        self.goal_tolerance = 0.12    # "Arrived" threshold
        self.reverse_tolerance = 0.10 # Close enough when reversing

        # P-gains
        # self.Kp_ang = ???

        # Safety clamps
        self.max_angular = 0.5

        # Logging
        self.log_counter = 0
        self.LOG_INTERVAL = 40  # Print every ~2 seconds

        # Timer
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('SMART NAVIGATOR | Waiting for GPS...')

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
            # Start navigating once we have GPS
            self.state = DRIVING

    def normalize_angle(self, angle):
        """Wrap angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def distance_to(self, x, y):
        """Euclidean distance from current pose to (x, y)."""
        return math.sqrt((x - self.pose.x)**2 + (y - self.pose.y)**2)

    def angle_to(self, x, y):
        """Angle from current pose to (x, y)."""
        return math.atan2(y - self.pose.y, x - self.pose.x)

    def get_current_waypoint(self):
        """Return the current target (x, y)."""
        return self.rows[self.current_row][self.current_wp]

    def get_row_end(self):
        """Return the last waypoint of the current row."""
        return self.rows[self.current_row][-1]

    def heading_of_next_row(self):
        """Calculate the heading direction of the next row."""
        if self.current_row + 1 >= len(self.rows):
            return self.pose.theta  # No next row
        row = self.rows[self.current_row + 1]
        return math.atan2(
            row[-1][1] - row[0][1],
            row[-1][0] - row[0][0]
        )

    def advance_waypoint(self):
        """Move to next waypoint. Returns True if row is complete."""
        self.current_wp += 1
        if self.current_wp >= len(self.rows[self.current_row]):
            return True  # Row complete
        return False

    def advance_to_next_row(self):
        """Move to the next row. Returns True if all rows are done."""
        self.current_row += 1
        self.current_wp = 0
        if self.current_row >= len(self.rows):
            return True  # All rows done
        return False

    def clamp_angular(self, angular):
        """Clamp angular velocity to safe range."""
        return max(-self.max_angular, min(self.max_angular, angular))

    # ==================================================================
    # CONTROL LOOP (State Machine)
    # ==================================================================

    def control_loop(self):
        if self.pose is None:
            return

        cmd = Twist()

        # ==========================================================
        # STATE: IDLE
        # ==========================================================
        if self.state == IDLE:
            # Publish zero velocity. Do nothing.
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # ==========================================================
        # STATE: DRIVING (Pure Pursuit)
        # ==========================================================
        elif self.state == DRIVING:
            target = self.get_current_waypoint()
            dist = self.distance_to(target[0], target[1])

            # Check: should we switch to APPROACH?
            if dist < self.approach_radius:
                self.state = APPROACH
                self.get_logger().info(
                    f'APPROACH | Near waypoint ({target[0]:.2f}, {target[1]:.2f})')
            else:
                # ==================================================
                # TODO 2: Implement Pure Pursuit steering.
                # ==================================================
                # Step A: Find the look-ahead point on the path.
                #   - Use self.Ld as the look-ahead distance.
                #   - For simplicity, you can just steer toward the
                #     current waypoint if it's farther than Ld,
                #     otherwise steer toward the next one.
                #
                # Step B: Calculate the angle to the look-ahead point.
                #   angle_to_target = self.angle_to(???, ???)
                #
                # Step C: Calculate the angle error.
                #   angle_error = self.normalize_angle(??? - self.pose.theta)
                #
                # Step D: Calculate curvature-based steering.
                #   curvature = 2.0 * math.sin(angle_error) / self.Ld
                #   cmd.linear.x = self.cruise_speed
                #   cmd.angular.z = self.clamp_angular(curvature * cmd.linear.x)
                #
                # Step E: If angle_error is large (> pi/2), slow down
                #         so the robot doesn't drive away from the goal.
                #   if abs(angle_error) > math.pi / 2:
                #       cmd.linear.x = ???  # Much slower
                pass

        # ==========================================================
        # STATE: APPROACH (Slow down near row end)
        # ==========================================================
        elif self.state == APPROACH:
            target = self.get_current_waypoint()
            dist = self.distance_to(target[0], target[1])

            if dist < self.goal_tolerance:
                # Arrived at waypoint
                row_done = self.advance_waypoint()
                if row_done:
                    # Row is complete
                    if self.current_row + 1 >= len(self.rows):
                        # All rows done!
                        self.state = IDLE
                        self.get_logger().info('ALL ROWS COMPLETE! IDLE.')
                    else:
                        # Start headland turn
                        self.state = HEADLAND
                        self.turn_phase = 0
                        self.get_logger().info('HEADLAND TURN | Starting U-turn')
                else:
                    # More waypoints in this row
                    self.state = DRIVING
                    self.get_logger().info(
                        f'DRIVING | Next waypoint in row {self.current_row}')
            else:
                # ==================================================
                # TODO 3: Implement APPROACH steering.
                # ==================================================
                # Same as DRIVING but use self.approach_speed instead
                # of self.cruise_speed. The robot should slow down
                # gently as it nears the row end.
                #
                # Hint: You can copy your DRIVING logic but with
                # a lower linear.x speed.
                pass

        # ==========================================================
        # STATE: HEADLAND (U-turn sequence)
        # ==========================================================
        elif self.state == HEADLAND:
            # ==================================================
            # TODO 4: Implement the headland turn.
            # ==================================================
            # This is a 3-phase scripted manoeuvre:
            #
            # Phase 0: Drive forward past the row end (overshoot).
            #   cmd.linear.x = self.turn_speed
            #   cmd.angular.z = 0.0
            #   Transition to phase 1 when you've driven 0.2-0.3m
            #   past the row end.
            #
            # Phase 1: Turn hard toward the next row.
            #   cmd.linear.x = small value (0.02-0.03)
            #   cmd.angular.z = max_angular (turn as hard as possible)
            #   Determine turn direction: which way to the next row?
            #   Transition to phase 2 when heading is within ~30 deg
            #   of the next row's heading.
            #
            # Phase 2: Straighten out.
            #   cmd.linear.x = self.turn_speed
            #   cmd.angular.z = gentle correction toward next row heading
            #   Transition to REVERSE (or DRIVING) when aligned
            #   within ~10 deg of the next row heading.
            #
            # Hint: Use self.heading_of_next_row() to know which
            # direction the next row goes.
            # Hint: Use self.normalize_angle() to compare headings.
            pass

        # ==========================================================
        # STATE: REVERSE (Back up to align with row entry)
        # ==========================================================
        elif self.state == REVERSE:
            # ==================================================
            # TODO 5: Implement reverse steering.
            # ==================================================
            # The robot needs to back up to the start of the next row.
            # Remember: steering is INVERTED when reversing!
            #
            # Step A: Get the entry point of the next row.
            #   entry = self.rows[self.current_row][0]
            #
            # Step B: Calculate distance to entry.
            #   dist = self.distance_to(entry[0], entry[1])
            #
            # Step C: If close enough, transition to DRIVING.
            #   if dist < self.reverse_tolerance:
            #       self.state = DRIVING
            #
            # Step D: Calculate angle to goal, but add pi
            #   (because we're going backward).
            #   angle_to_goal = self.angle_to(entry[0], entry[1])
            #   angle_error = self.normalize_angle(
            #       angle_to_goal - self.pose.theta + math.pi)
            #
            # Step E: Set reverse velocity.
            #   cmd.linear.x = -self.reverse_speed  # Negative!
            #   cmd.angular.z = self.clamp_angular(
            #       -self.Kp_ang * angle_error)  # Inverted!
            pass

        # ==========================================================
        # DIAGNOSTIC LOG
        # ==========================================================
        self.log_counter += 1
        if self.log_counter >= self.LOG_INTERVAL:
            self.log_counter = 0
            wp = self.get_current_waypoint() if self.current_row < len(self.rows) else (0, 0)
            self.get_logger().info(
                f'[{self.state}] Row {self.current_row} WP {self.current_wp} | '
                f'Pos: ({self.pose.x:+.2f}, {self.pose.y:+.2f}) '
                f'theta={math.degrees(self.pose.theta):+.1f} deg | '
                f'Target: ({wp[0]:.2f}, {wp[1]:.2f}) | '
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
