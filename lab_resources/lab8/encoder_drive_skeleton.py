#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class EncoderDrive(Node):
    def __init__(self):
        super().__init__('encoder_drive_node')

        # 1. Ask the user for input in the terminal before we start the ROS loop
        self.target_distance = float(input("Enter distance to travel in meters: "))

        # 2. Setup Publisher (Mouth) and Subscriber (Ears)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 3. Setup Timer (Heartbeat - runs at 10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # 4. Memory variables (State)
        self.start_x = None
        self.start_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.has_finished = False

        self.get_logger().info(f"Target set to {self.target_distance}m. Ready to drive.")

    def odom_callback(self, msg):
        """ This reflex runs every time the robot's wheels turn and update the /odom topic """
        
        # TODO 1: Read the current x and y position from the Odometry message
        # Hint: Real robots use nested messages. You need msg.pose.pose.position.x
        self.current_x = ... 
        self.current_y = ... 

        # Save the starting position the very first time we get a message
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y

    def control_loop(self):
        """ This is the brain that decides what to publish based on the memory """
        
        # If we haven't received sensor data yet, or we're already done, do nothing
        if self.start_x is None or self.has_finished:
            return

        # TODO 2: Calculate the Euclidean distance traveled so far
        # Formula: sqrt((x2 - x1)^2 + (y2 - y1)^2)
        # Hint: Use math.sqrt() and your self. variables!
        distance_traveled = ... 

        # Create a blank Twist message
        msg = Twist()

        # TODO 3: Write the logic to stop the robot
        # IF distance_traveled is LESS than self.target_distance: 
        #    Drive forward (e.g., set msg.linear.x = 0.15)
        # ELSE: 
        #    Stop the robot (set msg.linear.x = 0.0) AND set self.has_finished = True

        if ... :
            # Drive forward
            msg.linear.x = ...
        else:
            # Stop
            msg.linear.x = 0.0
            self.has_finished = True
            self.get_logger().info(f"Reached target! Traveled: {distance_traveled:.3f}m")

        # Publish the message to the motors
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderDrive()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()