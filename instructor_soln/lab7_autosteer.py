#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class Autosteer(Node):
    def __init__(self):
        super().__init__('autosteer_node')
        
        self.pub_cmd = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.pose = None
        
        # --- PATH PLANNING ---
        self.resolution = 0.1
        self.waypoints = self.generate_field_path()
        self.current_wp_index = 0
        
        # --- PID GAINS (Tuning) ---
        self.Kp_linear  = 2.0   
        self.Kp_angular = 4.0  
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"Plan Generated. {len(self.waypoints)} points created.")

    def generate_field_path(self):
            """ Builds the path segment-by-segment. """
            path = []
            
            # SEGMENT 1: DRIVE EAST (Row 1) 
            start_x, start_y = 1.0, 1.0
            end_x, end_y = 9.0, 1.0
            
            dist = 8.0
            num_steps = int(dist / self.resolution)
            
            for i in range(num_steps):
                alpha = i / num_steps
                x = start_x + (end_x - start_x) * alpha
                y = start_y + (end_y - start_y) * alpha
                path.append((x, y))
                
            # SEGMENT 2: THE J-TURN (First 90 degrees) 
            center_x = 9.0
            center_y = 1.5
            radius = 0.5
            num_turn_steps = 30
            
            for i in range(num_turn_steps):
                # From -90 deg (-1.57) to 0 deg
                angle = -1.57 + (1.57 * (i / num_turn_steps))
                x = center_x + radius * math.cos(angle)
                y = center_y + radius * math.sin(angle)
                path.append((x, y))
                
            # ---------------------------------------------------------
            # LAB CHALLENGE SOLUTIONS (Segments 3 & 4)
            # ---------------------------------------------------------

            # SEGMENT 3: Finish the U-Turn (Second 90 degrees)
            for i in range(num_turn_steps):
                # From 0 deg to +90 deg (+1.57 rad)
                angle = 0.0 + (1.57 * (i / num_turn_steps))
                x = center_x + radius * math.cos(angle)
                y = center_y + radius * math.sin(angle)
                path.append((x, y))

            # SEGMENT 4: Drive West (Return Trip on Row 2)
            start_x_ret, start_y_ret = 9.0, 2.0
            end_x_ret, end_y_ret = 1.0, 2.0
            
            # Calculate steps based on distance and resolution
            dist_ret = math.sqrt((end_x_ret - start_x_ret)**2 + (end_y_ret - start_y_ret)**2)
            num_steps_ret = int(dist_ret / self.resolution)

            for i in range(num_steps_ret):
                alpha = i / num_steps_ret
                x = start_x_ret + (end_x_ret - start_x_ret) * alpha
                y = start_y_ret + (end_y_ret - start_y_ret) * alpha
                path.append((x, y))
                
            return path

    def update_pose(self, msg): self.pose = msg

    def control_loop(self):
        if self.pose is None: return

        if self.current_wp_index >= len(self.waypoints):
            self.stop_robot(); return

        target_x, target_y = self.waypoints[self.current_wp_index]

        # 1. Calculate Errors
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.pose.theta

        # 2. Normalize Angle (Fix the 360 wrap)
        if angle_error > math.pi: angle_error -= 2 * math.pi
        elif angle_error < -math.pi: angle_error += 2 * math.pi

        msg = Twist()

        # 3. PID Control
        if distance > 0.1: # Tighter tolerance for curves
            msg.linear.x = min(2.0, self.Kp_linear * distance)
            msg.angular.z = self.Kp_angular * angle_error
        else:
            self.current_wp_index += 1 # Eat breadcrumb
            
        self.pub_cmd.publish(msg)

    def stop_robot(self):
        msg = Twist(); self.pub_cmd.publish(msg)
        self.get_logger().info("Mission Complete.")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Autosteer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()