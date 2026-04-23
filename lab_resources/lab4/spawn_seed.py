#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class Seeder(Node):
    def __init__(self):
        super().__init__('seeder')

        # 1. CREATE CLIENT: Open the app
        self.client = self.create_client(Spawn, '/spawn')

        # 2. CHECK FOR SERVER: Is the simulator running?
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for turtlesim /spawn service...')

        # 3. PREPARE THE ORDER (reusable request object)
        self.req = Spawn.Request()

    def plant(self, x, y, name):
        """Send a spawn request and wait for confirmation."""
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