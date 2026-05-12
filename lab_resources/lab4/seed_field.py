#!/usr/bin/env python3
"""
Lab 4, Part 5: Precision Seeder (3x4 grid of wheat).

For each grid point: teleport turtle1 to the target coordinate,
then spawn a seed at the same coordinate.

Calls: /turtle1/teleport_absolute  (turtlesim/srv/TeleportAbsolute)
Calls: /spawn                      (turtlesim/srv/Spawn)
"""
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, TeleportAbsolute


class FieldSeeder(Node):
    def __init__(self):
        super().__init__('field_seeder')

        # --- TWO SERVICE CLIENTS ---------------------------------------
        # TODO: create a client for '/spawn' (type Spawn)
        # self.spawn_client = ...

        # TODO: create a client for '/turtle1/teleport_absolute'
        #       (type TeleportAbsolute)
        # self.teleport_client = ...

        # TODO: wait_for_service on BOTH clients before continuing.

    def teleport(self, x, y):
        """Move turtle1 to (x, y) facing east. Blocks until done."""
        # TODO: build a TeleportAbsolute.Request(), fill x/y/theta,
        #       call_async, then spin_until_future_complete.
        # Safe to block here because we're called from main(), not
        # from inside a callback.
        pass

    def plant(self, x, y, name):
        """Spawn a seed at (x, y). Blocks until done."""
        # TODO: same pattern as teleport, but with Spawn.Request().
        pass


def main(args=None):
    rclpy.init(args=args)
    node = FieldSeeder()

    start_x, start_y = 1.0, 1.0
    spacing = 2.0

    for row in range(3):
        for col in range(4):
            x = start_x + col * spacing
            y = start_y + row * spacing
            try:
                node.teleport(x, y)
                node.plant(x, y, f"wheat_{row}_{col}")
                node.get_logger().info(f"Planted wheat_{row}_{col}")
            except Exception as e:
                node.get_logger().warn(
                    f"Failed at ({x},{y}): {e}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()