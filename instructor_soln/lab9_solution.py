import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class Lab9MasterSolution(Node):
    def __init__(self):
        super().__init__('lab9_master_solution')
        
        # 1. THE PUBLISHER: Sends math to your Lab 8 'uofw_bridge'
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. THE SUBSCRIBER: Listens to the God Node (Ceiling Camera) over Wi-Fi
        self.sub_gps = self.create_subscription(Pose, '/tractor/gps', self.update_pose, 10)
        
        self.pose = None
        
        # THE GOAL: Drive to the exact center of the camera's vision
        self.target_x = 0.0
        self.target_y = 0.0
        
        # TUNING GAINS: 
        # (These are kept low so the physical robot doesn't aggressively jitter)
        self.Kp_dist = 0.5 
        self.Kp_ang  = 1.2   
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Instructor Lab 9 PID Online. Waiting for God Node...")

    def update_pose(self, msg):
        """Callback triggers every time the camera sees the robot."""
        self.pose = msg

    def normalize_angle(self, angle):
        """Fixes the 360-degree wrap-around problem."""
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def control_loop(self):
        # Safety catch: Don't do math if the camera hasn't found us yet
        if self.pose is None: 
            return
        
        cmd = Twist()
        
        # ==========================================================
        # THE PID AUTOSTEER LOGIC
        # ==========================================================
        # Step A: Calculate the X and Y errors
        dx = self.target_x - self.pose.x
        dy = self.target_y - self.pose.y
        
        # Step B: Calculate Euclidean distance to target
        distance = math.sqrt(dx**2 + dy**2)
        
        # Step C: Calculate the angle the robot needs to face
        angle_to_goal = math.atan2(dy, dx)
        
        # Step D: Calculate the difference between current heading and goal heading
        angle_error = self.normalize_angle(angle_to_goal - self.pose.theta)
        
        # Step E & F: Proportional Control with a Stopping Condition
        if distance > 0.15: # 15cm deadzone so it doesn't violently micro-correct at the end
            # Capped at 0.25 m/s so it doesn't fly off the table
            cmd.linear.x = min(0.25, self.Kp_dist * distance) 
            cmd.angular.z = self.Kp_ang * angle_error
        else:
            # We have arrived. Slam the brakes.
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("Target Reached. Holding position.")
        
        # ==========================================================
        # HARDWARE SAFETY CLAMP
        # ==========================================================
        if cmd.angular.z > 0.5:
            cmd.angular.z = 0.5
        elif cmd.angular.z < -0.5:
            cmd.angular.z = -0.5

        self.pub_vel.publish(cmd)
        
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Lab9MasterSolution())
    rclpy.shutdown()

if __name__ == '__main__':
    main()