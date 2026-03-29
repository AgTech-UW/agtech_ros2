import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class Lab8MasterSolution(Node):
    def __init__(self):
        super().__init__('lab8_master_solution')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.state = 'FORWARD_1'
        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.target_distance = 3.0 # meters

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Instructor Solution Online. Waiting for Odometry...')

    def get_yaw(self, q):
        """Converts a Quaternion into a standard Euler Yaw angle."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.get_yaw(msg.pose.pose.orientation)

        # Lock in the starting coordinates the exact moment we get our first message
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw
            self.get_logger().info('Odometry locked. Starting 3m drive.')

    def control_loop(self):
            if self.start_x is None:
                return

            msg = Twist()

            if self.state == 'FORWARD_1':
                # Calculate distance from the start of FORWARD_1
                dist = math.sqrt((self.current_x - self.start_x)**2 + (self.current_y - self.start_y)**2)
                
                if dist < self.target_distance:
                    msg.linear.x = 0.15
                else:
                    self.get_logger().info(f'Reached {dist:.2f}m. Executing U-Turn.')
                    self.state = 'TURN'
                    self.start_yaw = self.current_yaw # Reset start yaw for the turn

            elif self.state == 'TURN':
                # Calculate absolute yaw difference
                yaw_diff = abs(self.current_yaw - self.start_yaw)
                
                # Handle the 360-degree wrap around
                if yaw_diff > math.pi: 
                    yaw_diff = (2 * math.pi) - yaw_diff

                # Turn until roughly ~162 degrees (0.90 * Pi) to account for momentum/drift
                if yaw_diff < math.pi * 0.90: 
                    msg.linear.x = 0.15  # Must move forward
                    msg.angular.z = 0.5  # CRITICAL FIX: Max servo limit is 0.5
                else:
                    self.get_logger().info('U-Turn complete. Driving back 3m.')
                    self.state = 'FORWARD_2'
                    self.start_x = self.current_x # Reset start position for the return trip
                    self.start_y = self.current_y

            elif self.state == 'FORWARD_2':
                # Calculate distance from the start of FORWARD_2
                dist = math.sqrt((self.current_x - self.start_x)**2 + (self.current_y - self.start_y)**2)
                
                if dist < self.target_distance:
                    msg.linear.x = 0.15
                else:
                    self.get_logger().info('Mission Complete. Stopping.')
                    self.state = 'DONE'

            elif self.state == 'DONE':
                msg.linear.x = 0.0
                msg.angular.z = 0.0

            # ==========================================
            # RESTORED HARDWARE CLAMP 
            # ==========================================
            if msg.angular.z > 0.5:
                msg.angular.z = 0.5
            elif msg.angular.z < -0.5:
                msg.angular.z = -0.5

            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Lab8MasterSolution())
    rclpy.shutdown()

if __name__ == '__main__':
    main()