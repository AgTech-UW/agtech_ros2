#!/usr/bin/env python3
"""
Lab 4 resource: Field Driver (snaking serpentine path).

This is your Lab 3 safety_stop, leveled up. Instead of one paperclip turn,
the driver snakes through the field for `num_rows` east/west sweeps:
    drive east -> hit fence -> arc up and over -> drive west -> hit fence ->
    arc up and over -> drive east -> ... -> stop after N rows.

The number of rows and the row spacing are ROS parameters, so you can
change behavior at launch time without editing this file:

    ros2 run lab4_services field_driver
    ros2 run lab4_services field_driver --ros-args -p num_rows:=2
    ros2 run lab4_services field_driver --ros-args -p num_rows:=6 \\
                                         -p row_spacing:=1.5

Subscribes: /turtle1/pose      (turtlesim/Pose)
Publishes:  /turtle1/cmd_vel   (geometry_msgs/Twist)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class FieldDriver(Node):
    def __init__(self):
        super().__init__('field_driver')

        # --- 1. PARAMETERS ----------------------------------------------
        # declare_parameter sets up a name + default. The value can be
        # overridden from the command line at launch time.
        self.declare_parameter('num_rows', 4)
        self.declare_parameter('row_spacing', 2.0)
        self.num_rows = self.get_parameter('num_rows').value
        self.row_spacing = float(self.get_parameter('row_spacing').value)

        # Pick turn rate so the vertical advance during a 180-degree arc
        # equals row_spacing. radius = linear / angular, advance = 2*radius.
        self.cruise_speed = 1.5
        self.turn_speed = 0.5
        self.turn_rate = (2.0 * self.turn_speed) / self.row_spacing

        self.get_logger().info(
            f"Field driver online: {self.num_rows} rows, "
            f"{self.row_spacing} m spacing, turn_rate={self.turn_rate:.2f}"
        )

        # --- 2. I/O ------------------------------------------------------
        self.publisher_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer_ = self.create_timer(0.1, self.timer_logic)

        # --- 3. STATE MACHINE -------------------------------------------
        # Five states: drive east, turn at east, drive west, turn at west,
        # done. We start by driving east.
        self.state = 'FORWARD_EAST'
        self.rows_completed = 0

    def timer_logic(self):
        msg = Twist()

        if self.state == 'FORWARD_EAST':
            msg.linear.x = self.cruise_speed
            msg.angular.z = 0.0

        elif self.state == 'TURN_AT_EAST':
            # Turning CCW (positive angular) takes us from east-facing
            # up over the top to west-facing. Robot drifts up during the
            # arc, which is what we want.
            msg.linear.x = self.turn_speed
            msg.angular.z = self.turn_rate

        elif self.state == 'FORWARD_WEST':
            msg.linear.x = self.cruise_speed
            msg.angular.z = 0.0

        elif self.state == 'TURN_AT_WEST':
            # Turning CW (negative angular) takes us from west-facing
            # up over the top to east-facing. Robot drifts up again.
            msg.linear.x = self.turn_speed
            msg.angular.z = -self.turn_rate

        elif self.state == 'DONE':
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)

    def pose_callback(self, msg):
        # Mission complete check first
        if self.rows_completed >= self.num_rows:
            if self.state != 'DONE':
                self.get_logger().info(
                    f"Field complete: {self.rows_completed} rows worked.")
                self.state = 'DONE'
            return

        if self.state == 'FORWARD_EAST' and msg.x > 9.0:
            self.state = 'TURN_AT_EAST'
            self.get_logger().info("East fence hit. Arcing.")

        elif self.state == 'TURN_AT_EAST' and abs(msg.theta) > 3.0:
            # theta near +/- pi means we're now facing west
            self.state = 'FORWARD_WEST'
            self.rows_completed += 1
            self.get_logger().info(
                f"Row {self.rows_completed}/{self.num_rows} done. "
                f"Heading west.")

        elif self.state == 'FORWARD_WEST' and msg.x < 1.0:
            self.state = 'TURN_AT_WEST'
            self.get_logger().info("West fence hit. Arcing.")

        elif self.state == 'TURN_AT_WEST' and abs(msg.theta) < 0.2:
            # theta near 0 means we're back to east-facing
            self.state = 'FORWARD_EAST'
            self.rows_completed += 1
            self.get_logger().info(
                f"Row {self.rows_completed}/{self.num_rows} done. "
                f"Heading east.")


def main(args=None):
    rclpy.init(args=args)
    node = FieldDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Always stop the wheels on shutdown. Same safety habit as Lab 3.
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()