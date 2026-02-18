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
        
        # --- 1. THE FIELD PLAN (A Single Row) ---
        # The farmer only gives us the start and end of Row 1.
        self.raw_waypoints = [ (1.0, 1.0), (9.0, 1.0) ]
        
        # --- 2. THE INTERPOLATOR (Breadcrumb Maker) ---
        self.resolution = 0.5 
        self.waypoints = self.densify_path(self.raw_waypoints, self.resolution)
        self.current_wp_index = 0
        
        # --- 3. PID GAINS (Tuning) ---
        self.Kp_linear  = 2.0   
        self.Kp_angular = 4.0  
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"Plan Generated. {len(self.waypoints)} points created.")

    def densify_path(self, raw_points, step_size):
        """ 
        Linear Interpolation: Draws straight lines between corners. 
        """
        dense_path = []
        
        for i in range(len(raw_points) - 1):
            p1 = raw_points[i]
            p2 = raw_points[i+1]
            
            dist_x = p2[0] - p1[0]
            dist_y = p2[1] - p1[1]
            dist = math.sqrt(dist_x**2 + dist_y**2)
            
            # How many breadcrumbs do we need?
            num_steps = max(1, int(dist / step_size))
            
            for j in range(num_steps):
                alpha = j / num_steps
                new_x = p1[0] + (dist_x * alpha)
                new_y = p1[1] + (dist_y * alpha)
                dense_path.append((new_x, new_y))
                
        # Don't forget the final point!
        dense_path.append(raw_points[-1])
        
        # --- PART 4 CHALLENGE ZONE ---
        # TODO: You will add your Headland Turn logic here later.
        
        return dense_path

    def update_pose(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.pose is None: return

        if self.current_wp_index >= len(self.waypoints):
            self.stop_robot()
            return

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

        # 3. PID Control (Tolerance: 0.2m)
        if distance > 0.2:
            # P-Controller: Speed is proportional to error
            msg.linear.x = min(2.0, self.Kp_linear * distance)
            msg.angular.z = self.Kp_angular * angle_error
        else:
            # We ate the breadcrumb! Move to next one.
            self.current_wp_index += 1 

        self.pub_cmd.publish(msg)

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0; msg.angular.z = 0.0
        self.pub_cmd.publish(msg)
        self.get_logger().info("Mission Complete.")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Autosteer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()