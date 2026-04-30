#!/usr/bin/env python3
"""
Lab 7: Autosteer - breadcrumb waypoint chasing with P-controller.

Subscribes: /turtle1/pose      (turtlesim/Pose)       - Robot position & heading
Publishes:  /turtle1/cmd_vel   (geometry_msgs/Twist)  - Velocity commands
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class Autosteer(Node):
    def __init__(self):
        super().__init__('autosteer_node')
        self.pub_cmd = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub_pose = self.create_subscription(
            Pose, '/turtle1/pose', self.update_pose, 10
        )
        self.pose = None

        # --- PATH PLANNING ---
        self.resolution = 0.1
        self.waypoints = self.generate_field_path()
        self.current_wp_index = 0

        # --- P-CONTROLLER GAINS (Tuning) ---
        self.Kp_linear = 2.0
        self.Kp_angular = 4.0

        # --- STATE FLAGS ---
        self.mission_done = False
        self.first_pose_received = False

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f"Plan generated. {len(self.waypoints)} waypoints created."
        )

    def generate_field_path(self):
        """
        Builds the path segment-by-segment.
        """
        path = []

        # --- SEGMENT 1: DRIVE EAST (Row 1) ---
        # From (1, 1) to (9, 1)
        start_x, start_y = 1.0, 1.0
        end_x, end_y = 9.0, 1.0
        dist = 8.0
        num_steps = int(dist / self.resolution)
        for i in range(num_steps + 1):  # +1 so the final waypoint hits (9, 1)
            # Linear interpolation formula
            alpha = i / num_steps
            x = start_x + (end_x - start_x) * alpha
            y = start_y + (end_y - start_y) * alpha
            path.append((x, y))

        # --- SEGMENT 2: THE J-TURN (First 90 degrees) ---
        # We turn LEFT (counter-clockwise) to face north.
        # Center of turn: (9.0, 1.5). Radius: 0.5.
        center_x = 9.0
        center_y = 1.5
        radius = 0.5
        # Arc math: we go from -pi/2 (bottom) to 0 (right)
        num_turn_steps = 30
        for i in range(num_turn_steps + 1):
            alpha = i / num_turn_steps
            angle = -math.pi / 2 + (math.pi / 2) * alpha
            # Circle parametric equation:
            # x = h + r*cos(theta)
            # y = k + r*sin(theta)
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            path.append((x, y))

        # --- STUDENT CHALLENGE ZONE ---
        # TODO 1: Add Segment 3 (Finish the U-Turn)
        # Hint: You need to go from 0 radians to +pi/2 radians.
        # TODO 2: Add Segment 4 (Drive West / Return to Start)
        # Hint: Copy Segment 1 logic but swap start/end coordinates.

        return path

    def update_pose(self, msg):
        if not self.first_pose_received:
            self.get_logger().info("First pose received - starting control.")
            self.first_pose_received = True
        self.pose = msg

    def control_loop(self):
        if self.pose is None:
            return

        # --- MISSION COMPLETE CHECK ---
        if self.current_wp_index >= len(self.waypoints):
            if not self.mission_done:
                self.get_logger().info("Mission complete.")
                self.mission_done = True
            self.stop_robot()
            return

        target_x, target_y = self.waypoints[self.current_wp_index]

        # 1. Calculate errors
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.pose.theta

        # 2. Normalize angle (fix the 360 wrap)
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        msg = Twist()

        # 3. P-Controller
        if distance > 0.1:  # Tight tolerance for curves
            msg.linear.x = min(2.0, self.Kp_linear * distance)
            msg.angular.z = self.Kp_angular * angle_error
        else:
            # Reached this breadcrumb - advance to next
            self.current_wp_index += 1

            # Log progress every 20 breadcrumbs so students can watch it work
            if self.current_wp_index % 20 == 0:
                self.get_logger().info(
                    f"Waypoint {self.current_wp_index}/"
                    f"{len(self.waypoints)}"
                )

        self.pub_cmd.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Autosteer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SAFETY: zero velocity before shutdown so the turtle doesn't
        # coast after Ctrl+C. You will use this pattern in every robot
        # controller this semester.
        stop_msg = Twist()
        node.pub_cmd.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()