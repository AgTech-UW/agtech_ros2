import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time

class JoySpoofer(Node):
    def __init__(self):
        super().__init__('joy_spoof_bridge')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        
        self.last_twist = Twist()
        self.timer = self.create_timer(0.05, self.publish_joy)
        self.get_logger().info('Optimized Joy Spoofer Online. [Press Ctrl+C for E-STOP]')

    def cmd_vel_callback(self, msg):
        self.last_twist = msg
        self.publish_joy()

    def publish_joy(self):
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        
        axes = [0.0] * 8
        axes[1] = self.last_twist.linear.x
        axes[3] = self.last_twist.angular.z 
        joy_msg.axes = axes
        
        buttons = [0] * 15
        buttons[5] = 1 # Deadman switch
        joy_msg.buttons = buttons
        
        self.publisher_.publish(joy_msg)

    def trigger_e_stop(self):
        """The absolute override. Sends flatline zeros to the motor controller."""
        self.get_logger().warn('*** E-STOP TRIGGERED! CUTTING MOTORS ***')
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        # Publish absolute zeros (No deadman switch, no velocity)
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 15 
        
        # Fire it 3 times rapidly just to guarantee the manufacturer node hears it
        for _ in range(3):
            self.publisher_.publish(joy_msg)
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    spoofer = JoySpoofer()
    
    try:
        rclpy.spin(spoofer)
    except KeyboardInterrupt:
        # THE INTERCEPT: Caught the Ctrl+C
        spoofer.trigger_e_stop()
    finally:
        spoofer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()