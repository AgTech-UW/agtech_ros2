import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose # Reusing this because students know it!
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class PhysicalGodNode(Node):
    def __init__(self):
        super().__init__('god_node')
        self.gps_pub = self.create_publisher(Pose, '/tractor/gps', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.broadcast_gps)
        
        self.get_logger().info("PHYSICAL GOD NODE ONLINE: Translating Camera TF to GPS...")

    def broadcast_gps(self):
        try:
            # Look up the tractor's physical tag relative to the arena center
            t = self.tf_buffer.lookup_transform('arena', 'tractor_tag', rclpy.time.Time())
            
            msg = Pose()
            msg.x = t.transform.translation.x
            msg.y = t.transform.translation.y
            
            # Quaternion to Yaw Math (Hidden from students!)
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            msg.theta = math.atan2(siny_cosp, cosy_cosp)

            self.gps_pub.publish(msg)

        except Exception as e:
            # Fails silently if the camera can't see the tag
            pass

def main():
    rclpy.init()
    rclpy.spin(PhysicalGodNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()