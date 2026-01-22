#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareDrawer(Node):
    def __init__(self):
        super().__init__('square_drawer_node')
        # Publisher: Sends messages to the turtle's velocity topic
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.move_turtle_logic)
        self.step = 0
        self.get_logger().info("Square Drawer Node has started!")

    def move_turtle_logic(self):
        # Create a message object (The "Letter" we are mailing)
        msg = Twist()

        # Logic: Drive for 2 seconds, Turn for 2 seconds
        if self.step % 2 == 0:
            msg.linear.x = 2.0  # Drive Forward
            msg.angular.z = 0.0
            self.get_logger().info("Driving Forward...")
        else:
            msg.linear.x = 0.0
            msg.angular.z = 1.57 # Turn 90 degrees (approx)
            self.get_logger().info("Turning...")

        # Publish the message (Mail the letter)
        self.publisher_.publish(msg)
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = SquareDrawer()
    rclpy.spin(node) # Keep the node running
    rclpy.shutdown()

if __name__ == '__main__':
    main()