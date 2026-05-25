#!/usr/bin/env python3
"""
=============================================================================
  MISSION RUNNER - Pure Pursuit Path Follower (Ackermann)
  AgTech ROS Course | PHYS-4000 / AG-3000 | Lab 10
=============================================================================
  Follows a multi-waypoint zigzag path using Pure Pursuit, the steering
  algorithm used in real autonomous vehicles. Designed for Ackermann-
  steered robots where minimum turning radius matters.

  Subscribes:
    /tractor/gps       (turtlesim/Pose) - Pose from the God Node

  Publishes:
    /auto/cmd_vel      (geometry_msgs/Twist) - Autonomous intent

  The Farm Manager arbitrates /auto/cmd_vel into /cmd_vel, gated by the
  keyboard spacebar deadman. This node NEVER writes directly to /cmd_vel.

  Usage:
    ros2 run lab10 mission_runner
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# --- TUNING ---
LOOK_AHEAD       = 0.30    # Pure Pursuit look-ahead distance (m)
CRUISE_SPEED     = 0.12    # Forward speed on rows (m/s)
TURN_SPEED       = 0.08    # Forward speed inside headland turns (m/s)
MAX_YAW_RATE     = 1.0     # Safety cap on angular.z (rad/s)
ARRIVE_TOLERANCE = 0.15    # Distance to last waypoint that counts as done (m)
TIGHT_CURVATURE  = 1.0     # Curvature threshold to switch to TURN_SPEED (1/m)

# --- HEADLAND GEOMETRY ---
# Your robot's min turning radius is ~0.35 m. Use 0.50 m for margin.
TURN_RADIUS      = 0.50


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def generate_zigzag_path():
    """Build a two-row zigzag with a single-arc headland turn.

    Returns a list of (x, y) breadcrumbs. Spacing is ~0.1 m, dense
    enough for Pure Pursuit to always find a look-ahead point.
    """
    path = []
    step = 0.1

    # --- Row 1: west to east at y = -TURN_RADIUS ---
    x_start, x_end = -1.0, 1.0
    y_row1 = -TURN_RADIUS
    n = int((x_end - x_start) / step)
    for i in range(n):
        alpha = i / n
        x = x_start + (x_end - x_start) * alpha
        path.append((x, y_row1))

    # --- Headland arc: 180 deg, centred at (x_end, y_row1 + TURN_RADIUS) ---
    # Same parametric-circle trick as Lab 7 Segment 2.
    cx, cy = x_end, y_row1 + TURN_RADIUS
    n_arc = 40
    # Angle starts at -pi/2 (bottom of circle) and sweeps to +pi/2 (top)
    for i in range(n_arc + 1):
        theta = -math.pi / 2 + (math.pi * i / n_arc)
        x = cx + TURN_RADIUS * math.cos(theta)
        y = cy + TURN_RADIUS * math.sin(theta)
        path.append((x, y))

    # --- Row 2: east to west at y = +TURN_RADIUS ---
    y_row2 = y_row1 + 2 * TURN_RADIUS
    for i in range(n):
        alpha = i / n
        x = x_end - (x_end - x_start) * alpha
        path.append((x, y_row2))

    return path


class MissionRunner(Node):
    def __init__(self):
        super().__init__('mission_runner')

        self.create_subscription(Pose, '/tractor/gps', self.on_pose, 10)
        self.pub = self.create_publisher(Twist, '/auto/cmd_vel', 10)

        self.pose = None
        self.path = generate_zigzag_path()
        self.closest_idx = 0
        self.mission_complete = False
        self.last_pose_time = self.get_clock().now()

        self.create_timer(0.05, self.tick)   # 20 Hz
        self.get_logger().info(
            f"Mission Runner ONLINE. {len(self.path)} breadcrumbs loaded."
        )

    def on_pose(self, msg: Pose):
        self.pose = msg
        self.last_pose_time = self.get_clock().now()

    # --- Pure Pursuit core ---
    def find_lookahead_target(self):
        """Find the first breadcrumb at least LOOK_AHEAD metres ahead.

        Also advances self.closest_idx so we don't scan the whole path
        every tick.
        """
        px, py = self.pose.x, self.pose.y

        # Advance closest_idx while the next breadcrumb is still within
        # look-ahead distance behind us.
        while self.closest_idx + 1 < len(self.path):
            nx, ny = self.path[self.closest_idx + 1]
            if math.hypot(nx - px, ny - py) < LOOK_AHEAD:
                self.closest_idx += 1
            else:
                break

        # Walk forward from closest_idx looking for a point at least L_d ahead.
        for i in range(self.closest_idx, len(self.path)):
            tx, ty = self.path[i]
            if math.hypot(tx - px, ty - py) >= LOOK_AHEAD:
                return (tx, ty, i)

        # Fell off the end. Return the last point.
        return (self.path[-1][0], self.path[-1][1], len(self.path) - 1)

    def pure_pursuit_cmd(self, target):
        """Compute (linear, angular) to drive toward the look-ahead point."""
        tx, ty, _ = target
        px, py, heading = self.pose.x, self.pose.y, self.pose.theta

        # Angle from robot heading to the look-ahead point.
        angle_to_target = math.atan2(ty - py, tx - px)
        alpha = normalize_angle(angle_to_target - heading)

        # Pure Pursuit curvature: kappa = 2 * sin(alpha) / L_d
        curvature = 2.0 * math.sin(alpha) / LOOK_AHEAD

        # Choose cruise vs turn speed based on how tight the curvature is.
        speed = TURN_SPEED if abs(curvature) > TIGHT_CURVATURE else CRUISE_SPEED

        # Yaw rate for Twist. Motor driver converts to steering angle.
        yaw_rate = clamp(curvature * speed, -MAX_YAW_RATE, MAX_YAW_RATE)

        return speed, yaw_rate

    def tick(self):
        cmd = Twist()

        if self.pose is None:
            self.pub.publish(cmd)
            return

        # GPS staleness check. If we haven't seen a pose update in 2 s,
        # something is wrong, publish zero and warn.
        age = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        if age > 2.0:
            self.get_logger().warn(
                f"GPS stale ({age:.1f}s old). Holding zero.",
                throttle_duration_sec=2.0)
            self.pub.publish(cmd)
            return

        if self.mission_complete:
            self.pub.publish(cmd)
            return

        # Done? (close enough to the last breadcrumb)
        last_x, last_y = self.path[-1]
        if math.hypot(last_x - self.pose.x,
                      last_y - self.pose.y) < ARRIVE_TOLERANCE:
            self.get_logger().info("Mission complete. Holding.")
            self.mission_complete = True
            self.pub.publish(cmd)
            return

        # Pure Pursuit control.
        target = self.find_lookahead_target()
        linear, angular = self.pure_pursuit_cmd(target)

        cmd.linear.x = linear
        cmd.angular.z = angular
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = MissionRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SAFETY: zero the auto command on shutdown.
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()