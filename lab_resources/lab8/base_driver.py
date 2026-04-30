import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class AutonomousDriver(Node):
    def __init__(self):
        super().__init__('autonomous_driver')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # --- SKELETON VARIABLES ---
        self.start_time = self.get_clock().now()
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # TODO 1: Add variables for your state machine (e.g., self.state = 'FORWARD_1')
        # TODO 2: Add variables to store your starting X, Y, and Yaw so you can measure distance traveled.
        #         Also add: self.turn_start_time = None

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Driver Online. Executing 1-second blind drive...')

    def get_yaw(self, q):
        """Converts a Quaternion into a standard Euler Yaw angle."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        """Updates internal variables with the latest odometry data."""
        # TODO 3: Extract the X and Y positions from the msg object.
        #         Hint: msg.pose.pose.position.x
        # TODO 4: Extract the orientation and pass it to self.get_yaw() to get your current angle.
        pass

    def control_loop(self):
        msg = Twist()

        # ==========================================================
        # CURRENT BEHAVIOR: 1-SECOND BLIND TIMER DRIVE
        # ==========================================================
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < 1.0:
            msg.linear.x = 0.15
        else:
            msg.linear.x = 0.0

        # ==========================================================
        # CHALLENGE: REPLACE THE ABOVE WITH YOUR STATE MACHINE
        # ==========================================================
        # TODO 5: Calculate Euclidean distance:
        #         d = sqrt((current_x - start_x)^2 + (current_y - start_y)^2)

        # TODO 6: Build the State Machine:
        # IF state == 'FORWARD_1':
        #     Drive forward until distance >= 3.0 meters. Switch state to 'TURN'.
        # ELIF state == 'TURN':
        #     Publish angular velocity (msg.angular.z) AND slight linear velocity (Ackermann constraint!)
        #     Use a timer to control turn duration rather than yaw, since the EKF on this
        #     hardware overcounts angular displacement. Tune the duration experimentally.
        #     Hint: initialize self.turn_start_time when entering TURN, then check:
        #           turn_elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
        # ELIF state == 'FORWARD_2':
        #     Drive forward until you return to the origin.
        # ELIF state == 'DONE':
        #     Stop all motors.

        # HARDWARE SAFETY CLAMP (DO NOT REMOVE)
        if msg.angular.z > 0.5:
            msg.angular.z = 0.5
        elif msg.angular.z < -0.5:
            msg.angular.z = -0.5

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AutonomousDriver())
    rclpy.shutdown()

if __name__ == '__main__':
    main()