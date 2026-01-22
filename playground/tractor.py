import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from std_msgs.msg import Empty
import math
import time

# STATES
STATE_EAST       = 0
STATE_TURN_RIGHT = 1
STATE_WEST       = 2
STATE_TURN_LEFT  = 3

class SnakeTractor(Node):
    def __init__(self):
        super().__init__('snake_tractor')
        
        self.pub_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_seed = self.create_publisher(Empty, '/tractor/deploy_seed', 10)
        self.sub_gps = self.create_subscription(Pose, '/tractor/gps', self.update_pose, 10)
        
        # Clients for Teleport and Pen Control
        self.client_teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.client_pen = self.create_client(SetPen, '/turtle1/set_pen')
        
        self.pose = None
        self.state = STATE_EAST
        self.target_y = 10.0 # Start at the Top
        self.seed_timer = 0
        
        # TUNING
        self.Kp_dist = 2.0 
        self.Kp_ang  = 2.0   
        
        # AUTOMATIC SETUP
        print("🚜 TRACTOR ONLINE. Setting up field...")
        self.setup_field() 
        
        # Start the brain only AFTER setup
        self.timer = self.create_timer(0.05, self.control_loop)

    def set_pen(self, off=False):
        # Helper to turn pen on/off
        req = SetPen.Request()
        req.off = off
        req.r = 255; req.g = 255; req.b = 255; req.width = 2 # White line
        future = self.client_pen.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def setup_field(self):
        # Wait for services
        while not self.client_teleport.wait_for_service(timeout_sec=1.0):
            print("Waiting for Turtlesim...")

        # 1. Lift Pen (No ugly line)
        self.set_pen(off=True)

        # 2. Teleport to Start
        req = TeleportAbsolute.Request()
        req.x = 1.0
        req.y = 10.0
        req.theta = 0.0
        future = self.client_teleport.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        # 3. Put Pen Down (Ready to draw)
        self.set_pen(off=False)
        
        print("✅ Robot Ready at Start (X=1.0, Y=10.0)")

    def update_pose(self, msg):
        self.pose = msg

    def normalize_angle(self, angle):
        while angle > 3.14159: angle -= 2 * 3.14159
        while angle < -3.14159: angle += 2 * 3.14159
        return angle

    def control_loop(self):
        if self.pose is None: return
        
        cmd = Twist()
        
        # --- STATE 0: DRIVE EAST ---
        if self.state == STATE_EAST:
            cmd.linear.x = 1.0
            error_y = self.pose.y - self.target_y
            error_angle = self.normalize_angle(self.pose.theta - 0.0)
            
            cmd.angular.z = -2.0 * error_y - 1.0 * error_angle
            
            if self.pose.x > 9.0:
                self.state = STATE_TURN_RIGHT
                self.target_y -= 1.0 
            
            # Seeding
            if abs(error_y) < 0.2:
                self.seed_timer += 1
                if self.seed_timer > 10: self.pub_seed.publish(Empty()); self.seed_timer = 0

        # --- STATE 1: TURN RIGHT ---
        elif self.state == STATE_TURN_RIGHT:
            cmd.linear.x = 0.25   
            cmd.angular.z = -0.5 
            
            # WIDENED TOLERANCE (No more 360 spins)
            # 3.1 rad is ~177 degrees. Safe and precise.
            if abs(self.pose.theta) > 3.1: 
                self.state = STATE_WEST

        # --- STATE 2: DRIVE WEST ---
        elif self.state == STATE_WEST:
            cmd.linear.x = 1.0
            error_y = self.pose.y - self.target_y
            error_angle = self.normalize_angle(self.pose.theta - 3.14159)
            
            # Inverted P-Controller Logic for West
            correction_dist = +2.0 * error_y 
            correction_ang  = -1.0 * error_angle
            cmd.angular.z = correction_dist + correction_ang
            
            if self.pose.x < 2.0:
                self.state = STATE_TURN_LEFT
                self.target_y -= 1.0 

            # Seeding
            if abs(error_y) < 0.2:
                self.seed_timer += 1
                if self.seed_timer > 10: self.pub_seed.publish(Empty()); self.seed_timer = 0

        # --- STATE 3: TURN LEFT ---
        elif self.state == STATE_TURN_LEFT:
            cmd.linear.x = 0.25   
            cmd.angular.z = 0.5  
            
            # THE FIX: Tighten from 0.1 to 0.05
            # Now both turns have the same high precision (~2.8 degrees)
            if abs(self.pose.theta) < 0.05: 
                self.state = STATE_EAST

        self.pub_vel.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(SnakeTractor())

if __name__ == '__main__':
    main()