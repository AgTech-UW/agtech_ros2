import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class HardwareTractor(Node):
    def __init__(self):
        super().__init__('hardware_tractor')
        
        # 1. THE PUBLISHER: Sends math to your Lab 8 'uofw_bridge'
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. THE SUBSCRIBER: Listens to the God Node for our location
        self.sub_gps = self.create_subscription(Pose, '/tractor/gps', self.update_pose, 10)
        
        self.pose = None
        
        # THE GOAL: Drive to the center of the arena
        self.target_x = 0.0
        self.target_y = 0.0
        
        # TODO 1: Add your Kp tuning variables from Lab 7 here. 
        # (Hint: Physical hardware usually needs lower gains than Turtlesim!)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Tractor PID Online. Waiting for GPS...")

    def update_pose(self, msg):
        self.pose = msg

    def normalize_angle(self, angle):
        """Fixes the 360-degree wrap-around problem."""
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def control_loop(self):
        # Don't do math if we haven't seen the God Node yet
        if self.pose is None: 
            return
        
        cmd = Twist()
        
        # ==========================================================
        # TODO 2: THE PID AUTOSTEER LOGIC
        # ==========================================================
        # Step A: Calculate the X and Y errors.
        # Step B: Calculate the Euclidean distance to the target.
        # Step C: Calculate the angle_to_goal (using math.atan2).
        # Step D: Calculate the angle_error and normalize it.
        # Step E: Write the P-Controller logic to set cmd.linear.x and cmd.angular.z
        #         (Make sure to cap your max speed so the robot doesn't fly off the table!)
        # Step F: Write an IF statement to stop the robot if the distance is < 0.15 meters.
        
        
        self.pub_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HardwareTractor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()