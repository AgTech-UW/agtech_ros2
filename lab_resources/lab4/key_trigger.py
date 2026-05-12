#!/usr/bin/env python3
"""
Lab 4 helper: Keyboard Trigger.

Publishes std_msgs/Empty to /plant_seed on SPACE. Press 'q' to quit.
If your terminal looks weird after a crash, run `reset`.
"""
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class KeyTrigger(Node):
    def __init__(self):
        super().__init__('key_trigger')

        self.publisher_ = self.create_publisher(Empty, '/plant_seed', 10)

        # Poll the keyboard 20 times a second. Plenty fast for a human.
        self.timer_ = self.create_timer(0.05, self.read_key)

        # Stash the original terminal settings so we can restore them
        # cleanly when the node shuts down.
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.get_logger().info(
            "Key trigger ready. SPACE = plant a seed. q = quit.")

    def read_key(self):
        # Non-blocking peek: only read if a key is actually waiting.
        # Without this, sys.stdin.read(1) would block the executor.
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == ' ':
                self.publisher_.publish(Empty())
                self.get_logger().info("SPACE -> /plant_seed")
            elif ch == 'q':
                self.get_logger().info("Quit requested.")
                rclpy.shutdown()

    def restore_terminal(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()