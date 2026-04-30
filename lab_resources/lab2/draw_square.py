#!/usr/bin/env python3
"""
Lab 2: Square Drawer.
 
Publishes: /turtle1/cmd_vel (geometry_msgs/Twist)  - Velocity commands
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquareDrawer(Node):
    def __init__(self):
        # Give the node a name for the logs
        super().__init__('square_drawer_node')

        # 1. PUBLISHER: Speak "Twist" messages to '/turtle1/cmd_vel'
        self.publisher_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)

        # 2. TIMER: Wake up 'move_turtle_logic' every 1.0 seconds
        self.timer = self.create_timer(1.0, self.move_turtle_logic)

        self.step = 0
        self.get_logger().info("Square Drawer Node has started!")

    def move_turtle_logic(self):
        # Create a new message to send
        msg = Twist()

        # Logic: Even steps = Drive, Odd steps = Turn
        # This uses the Modulo operator (%) to check for remainders
        if self.step % 2 == 0:
            msg.linear.x = 2.0    # Drive forward (m/s)
            msg.angular.z = 0.0
            self.get_logger().info("Driving forward...")
        else:
            msg.linear.x = 0.0
            msg.angular.z = 1.57  # Turn 90 degrees (approx) in radians
            self.get_logger().info("Turning...")

        # Send the message using the publisher we saved in 'self'
        self.publisher_.publish(msg)
        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    node = SquareDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SAFETY: publish zero velocity on shutdown so the turtle
        # doesn't coast after Ctrl+C. You will use this pattern in
        # every robot controller you write this semester.
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()