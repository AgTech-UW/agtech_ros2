#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn 

class CropSpawner(Node):
    def __init__(self):
        super().__init__('crop_spawner')
        self.client = self.create_client(Spawn, '/spawn')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Simulator...')
            
        # FIRE THE PLANTING LOOP
        self.plant_row()

    def plant_row(self):
        # We will plant 5 crops in a line
        for i in range(5):
            request = Spawn.Request()
            
            # X = 2.0, 3.0, 4.0, 5.0, 6.0
            request.x = float(i + 2) 
            request.y = 8.0           # Keep them high up in the field
            request.theta = 0.0
            
            # Unique names are REQUIRED (crop_0, crop_1, etc.)
            request.name = f'crop_{i}' 
            
            # Send the order (Async = Don't wait for the receipt)
            self.client.call_async(request)
            self.get_logger().info(f"Ordered {request.name} at x={request.x}")

def main(args=None):
    rclpy.init(args=args)
    node = CropSpawner()
    
    # We only need to spin briefly to let the messages fly out
    # A short sleep ensures the network buffer clears
    try:
        rclpy.spin_once(node, timeout_sec=2.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()