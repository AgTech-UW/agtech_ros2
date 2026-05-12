#!/usr/bin/env python3
"""
Lab 4, Part 6 (Stage 1): Auto-Seeder.

This node is the centerpiece of Lab 4. It combines all three patterns
you have learned this semester:

  - Subscriber  (Lab 3)  on /turtle1/pose       -> remember the tractor's pose
  - Timer       (Lab 2)  every 3 seconds        -> fire the planting logic
  - Service Client (Lab 4) on /spawn            -> plant a seed at that pose

Subscribes: /turtle1/pose     (turtlesim/Pose)
Calls:      /spawn            (turtlesim/srv/Spawn)

YOUR JOB: fill in every line marked TODO. The structure is laid out for
you; you only need to add the ROS-specific calls. Refer back to:
  - safety_stop.py from Lab 3  (subscriber + timer pattern)
  - spawn_seed.py  from Lab 4  (service client pattern)
"""
import rclpy
from rclpy.node import Node

# TODO: import the message type for the pose subscriber.
# Hint: it's the same one you used in Lab 3's safety_stop.py.


# TODO: import the service type for the /spawn client.
# Hint: it's the same one you used earlier today in spawn_seed.py.



class AutoSeeder(Node):
    def __init__(self):
        super().__init__('auto_seeder')

        # --- 1. SUBSCRIBER (the ears) -----------------------------------
        # We need to know where turtle1 is RIGHT NOW so we can plant a
        # seed at that location.
        #
        # TODO: create a subscription to '/turtle1/pose' with a queue
        # depth of 10 and a callback of self.pose_callback.
        #
        # self.subscription_ = self.create_subscription( ... )

        # --- 2. SERVICE CLIENT (the mouth-with-confirmation) ------------
        # TODO: create a client for the '/spawn' service.
        #
        # self.spawn_client = self.create_client( ... )
        #
        # TODO: wait for the simulator to be running before we try to
        # call it. Loop on wait_for_service(timeout_sec=1.0). Log a
        # message each time we wait so the user knows what's happening.
        #
        # while not self.spawn_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for /spawn service...')

        # --- 3. TIMER (the heartbeat) -----------------------------------
        # TODO: create a timer that fires every 3.0 seconds and calls
        # self.timer_callback.
        #
        # self.timer_ = self.create_timer( ... )

        # --- 4. STATE ---------------------------------------------------
        # The latest pose we heard about. None until the first pose
        # message arrives. The timer has to handle that case.
        self.latest_pose = None

        # Counter so each seed gets a unique name. Re-using a name will
        # make /spawn return an empty response (the seed won't appear).
        self.seed_count = 0

        self.get_logger().info("Auto-seeder ready. Waiting for tractor...")

    def pose_callback(self, msg):
        """Cache the latest pose. Runs every time turtle1 publishes."""
        # This is the simplest callback you'll write all spring:
        # just remember what we heard. The timer does the work.
        self.latest_pose = msg

    def timer_callback(self):
        """Fires every 3 s. Plant a seed at whatever pose we last saw."""
        # Guard: skip if we haven't heard from the tractor yet.
        if self.latest_pose is None:
            self.get_logger().warn("No pose yet. Skipping plant.")
            return

        # TODO: build a Spawn.Request() and fill in:
        #   req.x     = self.latest_pose.x
        #   req.y     = self.latest_pose.y
        #   req.theta = 0.0
        #   req.name  = f"auto_seed_{self.seed_count}"
        #
        # req = Spawn.Request()
        # req.x     = ...
        # req.y     = ...
        # req.theta = ...
        # req.name  = ...

        # TODO: send the request asynchronously and attach a done
        # callback. DO NOT use spin_until_future_complete here -- we
        # are inside a callback already, and that would deadlock the
        # single-threaded executor (see Critical Warning in Part 3).
        #
        # future = self.spawn_client.call_async(req)
        # future.add_done_callback(self.spawn_done)

        self.seed_count += 1

    def spawn_done(self, future):
        """Runs whenever /spawn confirms (or fails)."""
        try:
            response = future.result()
            self.get_logger().info(f"Planted: {response.name}")
        except Exception as e:
            self.get_logger().error(f"Plant failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AutoSeeder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()