import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class GodNode(Node):
    def __init__(self):
        super().__init__('god_node')
        
        # Input: Truth Data from Sim
        self.sim_sub = self.create_subscription(Pose, '/turtle1/pose', self.handle_sim_pose, 10)
        
        # Output: GPS Data for Tractor
        self.gps_pub = self.create_publisher(Pose, '/tractor/gps', 10)
        
        print("GOD NODE ONLINE: GPS System Active (No Viz).")

    def handle_sim_pose(self, msg):
        # Relay the position
        self.gps_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(GodNode())

if __name__ == '__main__':
    main()