import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class HardwareTractor(Node):
    def __init__(self):
        super().__init__('hardware_tractor')
        
        # Publish to cmd_vel -> The Lab 8 Bridge catches this!
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscribe to the God Node's easy GPS data
        self.sub_gps = self.create_subscription(Pose, '/tractor/gps', self.update_pose, 10)
        
        self.pose = None
        
        # Hardcoded Goal: Drive to the center of the arena
        self.target_x = 0.0
        self.target_y = 0.0
        
        self.Kp_dist = 0.5 # Tuning for real hardware
        self.Kp_ang  = 1.0   
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Tractor PID Online. Waiting for GPS...")

    def update_pose(self, msg):
        self.pose = msg

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def control_loop(self):
        if self.pose is None: return
        
        cmd = Twist()
        
        # Calculate Error
        dx = self.target_x - self.pose.x
        dy = self.target_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        angle_to_goal = math.atan2(dy, dx)
        error_angle = self.normalize_angle(angle_to_goal - self.pose.theta)
        
        # Hardware PID Control
        if distance > 0.15: # Stop if within 15cm of goal
            cmd.linear.x = min(0.3, self.Kp_dist * distance) # Max speed 0.3 m/s
            cmd.angular.z = self.Kp_ang * error_angle
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("Waypoint Reached!")

        self.pub_vel.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(HardwareTractor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()