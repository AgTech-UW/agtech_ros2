import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class FakeGodNode(Node):
    def __init__(self):
        super().__init__('fake_god_node')
        self.gps_pub = self.create_publisher(Pose, '/tractor/gps', 10)
        self.timer = self.create_timer(0.1, self.broadcast_fake_gps)
        self.get_logger().info("FAKE GOD NODE ONLINE: Broadcasting Static Error...")

    def broadcast_fake_gps(self):
        msg = Pose()
        # Pretend the robot is 1 meter forward, 0.5 meters left, facing slightly right
        msg.x = 1.0
        msg.y = 0.5
        msg.theta = -0.2 
        self.gps_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(FakeGodNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()