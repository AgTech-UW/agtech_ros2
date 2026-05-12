#!/usr/bin/env python3
"""
Lab 4: Field Driver. Snakes through the field for N rows.

Subscribes: /turtle1/pose
Publishes:  /turtle1/cmd_vel
Calls:      /turtle1/teleport_absolute  (once, on startup)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

# Tunables -- tweak these to change behavior
NUM_ROWS     = 6
ROW_SPACING  = 1.5
START_X      = 1.0
START_Y      = 1.0
CRUISE_SPEED = 1.5
TURN_SPEED   = 0.5


class FieldDriver(Node):
    def __init__(self):
        super().__init__('field_driver')

        # Turn rate so a 180-degree arc advances exactly one row.
        self.turn_rate = (2.0 * TURN_SPEED) / ROW_SPACING

        # State init MUST come before the subscriber -- pose_callback
        # can fire during the teleport wait below.
        self.state = 'FORWARD_EAST'
        self.rows_completed = 0

        self.publisher_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        # Teleport to the starting corner so the snake fills the field.
        # Safe to block here: we're in __init__, no callbacks spinning.
        self.teleport_client = self.create_client(
            TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')

        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = START_X, START_Y, 0.0
        future = self.teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        self.timer_ = self.create_timer(0.1, self.timer_logic)
        self.get_logger().info(
            f"Field driver online. Working {NUM_ROWS} rows.")

    def timer_logic(self):
        msg = Twist()
        if self.state == 'FORWARD_EAST':
            msg.linear.x = CRUISE_SPEED
        elif self.state == 'TURN_AT_EAST':
            msg.linear.x = TURN_SPEED
            msg.angular.z = self.turn_rate
        elif self.state == 'FORWARD_WEST':
            msg.linear.x = CRUISE_SPEED
        elif self.state == 'TURN_AT_WEST':
            msg.linear.x = TURN_SPEED
            msg.angular.z = -self.turn_rate
        # DONE: leave msg zeroed
        self.publisher_.publish(msg)

    def pose_callback(self, msg):
        if self.state == 'DONE':
            return

        if self.state == 'FORWARD_EAST' and msg.x > 9.0:
            self.rows_completed += 1
            if self.rows_completed >= NUM_ROWS:
                self.state = 'DONE'
                self.get_logger().info(
                    f"Field complete: {self.rows_completed} rows.")
            else:
                self.state = 'TURN_AT_EAST'

        elif self.state == 'TURN_AT_EAST' and abs(msg.theta) > 3.1:
            self.state = 'FORWARD_WEST'

        elif self.state == 'FORWARD_WEST' and msg.x < 2.0:
            self.rows_completed += 1
            if self.rows_completed >= NUM_ROWS:
                self.state = 'DONE'
                self.get_logger().info(
                    f"Field complete: {self.rows_completed} rows.")
            else:
                self.state = 'TURN_AT_WEST'

        elif self.state == 'TURN_AT_WEST' and abs(msg.theta) < 0.04:
            self.state = 'FORWARD_EAST'


def main(args=None):
    rclpy.init(args=args)
    node = FieldDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher_.publish(Twist())  # safety stop
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()