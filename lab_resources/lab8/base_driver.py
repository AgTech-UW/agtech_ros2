import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class OdomDriver(Node):
    def __init__(self):
        super().__init__('odom_driver')
        # We publish commands for your bridge to pick up
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # We subscribe to the free Odometry the manufacturer node is providing
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.state = 'FORWARD_1'
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.target_distance = 3.0 # Meters
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Autonomous Driver Online: Waiting for Odometry...')

    def get_yaw(self, q):
        # Convert quaternion to Euler angle (Yaw)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.get_yaw(msg.pose.pose.orientation)

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw
            self.get_logger().info('Odometry locked. Starting 3m drive.')

    def control_loop(self):
        if self.start_x is None:
            return

        msg = Twist()
        # Calculate distance: d = sqrt((x2-x1)^2 + (y2-y1)^2)
        dist = math.sqrt((self.current_x - self.start_x)**2 + (self.current_y - self.start_y)**2)

        if self.state == 'FORWARD_1':
            if dist < self.target_distance:
                msg.linear.x = 0.3  # Drive forward at 0.3 m/s
            else:
                self.get_logger().info(f'Reached {dist:.2f}m. Executing U-Turn.')
                self.state = 'TURN'
                self.start_yaw = self.current_yaw

        elif self.state == 'TURN':
            # Calculate how far we've turned
            yaw_diff = abs(self.current_yaw - self.start_yaw)
            if yaw_diff > math.pi: yaw_diff -= 2 * math.pi
            if yaw_diff < -math.pi: yaw_diff += 2 * math.pi

            if abs(yaw_diff) < math.pi * 0.90: # Turn until we are roughly 180 degrees around
                msg.linear.x = 0.2  # Must move forward to steer
                msg.angular.z = 1.0 # Hard left turn
            else:
                self.get_logger().info('U-Turn complete. Driving back 3m.')
                self.state = 'FORWARD_2'
                self.start_x = self.current_x
                self.start_y = self.current_y

        elif self.state == 'FORWARD_2':
            if dist < self.target_distance:
                msg.linear.x = 0.3
            else:
                self.get_logger().info('Mission Complete. Stopping.')
                self.state = 'DONE'

        elif self.state == 'DONE':
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OdomDriver())
    rclpy.shutdown()

if __name__ == '__main__':
    main()