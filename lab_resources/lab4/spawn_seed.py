#!/usr/bin/env python3
"""
Lab 4: Seeder (service client).

Calls: /spawn (turtlesim/srv/Spawn)  -  Plants a marker turtle at (x, y)

Demonstrates the basic Service Client pattern:
  1. Create a client for a specific service type
  2. Wait for the service to become available
  3. Build a request, send it asynchronously, wait for the response
"""
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class Seeder(Node):
    def __init__(self):
        super().__init__('seeder')

        # 1. CREATE CLIENT: Open the "app" connection to /spawn
        self.client = self.create_client(Spawn, '/spawn')

        # 2. CHECK FOR SERVER: Is the simulator running?
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for turtlesim /spawn service...')

        # 3. PREPARE THE ORDER (reusable request object)
        self.req = Spawn.Request()

    def plant(self, x, y, name):
        """Send a spawn request and wait for confirmation."""
        # Fill out the order form
        self.req.x = float(x)
        self.req.y = float(y)
        self.req.theta = 0.0
        self.req.name = name

        # ASYNC CALL: Returns a Future (tracking number) instantly
        future = self.client.call_async(self.req)

        # WAIT FOR DELIVERY: Block until the simulator confirms
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = Seeder()

    # Plant a short row of 3 seeds across the middle of the field
    for i in range(1, 4):
        node.get_logger().info(f"Planting seed {i}...")
        try:
            # Calculate x to space them out: (2, 4, 6)
            response = node.plant(float(i * 2), 5.0, f"seed_{i}")
            node.get_logger().info(f"Success! Created: {response.name}")
        except Exception as e:
            node.get_logger().error(f"Failed: {e}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()