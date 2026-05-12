#!/usr/bin/env python3
"""
Lab 5, Part 7: The Geofence Sentinel.

This node monitors the mathematical relationship between the camera
and the tractor using the tf2_ros library. If the tractor strays
too far from the centre, it warns the system.

YOUR JOB: fill in every line marked TODO. The tf2 listener boilerplate
is provided so you can focus on the spatial math.
"""
import rclpy
from rclpy.node import Node
import math

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class GeofenceSentinel(Node):
    def __init__(self):
        super().__init__('geofence_sentinel')

        # Set up the TF2 Buffer and Listener.
        # This allows the node to automatically track coordinate frames.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to check the geofence 10 times per second.
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info("Geofence Sentinel ready. Watching tractor...")

    def on_timer(self):
        # 1. Ask TF2 for the relationship between camera and tag
        try:
            t = self.tf_buffer.lookup_transform(
                'default_cam',
                'tag36h11:0',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info('Waiting for tag detection...', throttle_duration_sec=2.0)
            return

        # 2. Extract X and Y translation (in metres)
        x = t.transform.translation.x
        y = t.transform.translation.y

        # TODO: Calculate Planar Distance
        # Use math.sqrt() to calculate the Euclidean distance (r) from the
        # camera centre (0,0) using x and y.
        #
        # r = ...

        # TODO: Enforce the Geofence
        # The safe zone is a circle with a radius of 0.40 metres.
        # If r is less than or equal to 0.40:
        #     Log an INFO message: "Tractor in safe zone. Distance: [r] m"
        # If r is greater than 0.40:
        #     Log a WARN message: "CRITICAL: Tractor out of bounds!"
        #
        # if ...

def main(args=None):
    rclpy.init(args=args)
    node = GeofenceSentinel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()