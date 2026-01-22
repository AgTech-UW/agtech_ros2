#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose # < CRITICAL IMPORT! 

class SafetyBrake(Node):
    def __init__(self):
        super().__init__('safety_brake_node')
        
        # 1. THE MOUTH (Publisher)
        # We send velocity commands to the turtle
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        
        # 2. THE EARS (Subscriber)
        # We listen to the turtle's position to check for the fence
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        ) 
        
        # 3. THE HEARTBEAT (Timer)
        # This wakes up 10 times a second to send commands
        self.timer = self.create_timer(0.1, self.timer_logic) 
        
        # 4. THE MEMORY (State)
        self.stop_robot = False
        self.get_logger().info("Geofence System Active") 

    def timer_logic(self):
        # The Brain: Decide WHAT to do based on memory
        msg = Twist()
        
        if self.stop_robot == True:
            msg.linear.x = 0.0 # E-BRAKE 
            msg.angular.z = 0.0
        else:
            msg.linear.x = 1.0 # Cruise Speed 
            msg.angular.z = 0.0
            
        self.publisher_.publish(msg)

    def pose_callback(self, msg):
        # The Reflex: Update memory based on senses
        if msg.x > 9.0: # The invisible wall 
            if not self.stop_robot:
                self.get_logger().warn(f"GEOFENCE BREACH AT X={msg.x:.2f}! STOPPING.") 
                self.stop_robot = True
        else:
            self.stop_robot = False

def main(args=None):
    rclpy.init(args=args)
    node = SafetyBrake()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()