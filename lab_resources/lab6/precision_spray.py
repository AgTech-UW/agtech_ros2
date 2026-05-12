#!/usr/bin/env python3
"""
Lab 6, Part 7: Precision Spraying.

This node uses computer vision to find a red marker, calculates its
centre, and triggers a spray signal ONLY if the weed is within a 
specific "treatment zone" in the centre of the camera frame.

Subscribes: /image_raw            (sensor_msgs/Image)
Publishes:  /weed_detector/debug  (sensor_msgs/Image)
Publishes:  /tractor/spray        (std_msgs/Bool)

YOUR JOB: fill in every line marked TODO. Focus on the spatial logic
and OpenCV drawing functions.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class PrecisionSpray(Node):
    def __init__(self):
        super().__init__('precision_spray')

        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        
        self.pub_debug = self.create_publisher(
            Image, '/weed_detector/debug', 10)
        self.pub_spray = self.create_publisher(
            Bool, '/tractor/spray', 10)

        self.bridge = CvBridge()
        
        self.get_logger().info("Precision Spray ready. Looking for targets...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # --- UPDATE THIS WITH YOUR CALIBRATED COLOURS ---
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # TODO: Find the centre of the camera frame
        # Hint: cv_image.shape returns a tuple (height, width, channels)
        #
        # frame_height = ...
        # frame_width = ...
        # frame_center_x = int(...)
        # frame_center_y = int(...)

        # TODO: Define the 100x100 pixel "Treatment Zone"
        # Calculate the top-left and bottom-right corners of a 100x100 box
        # centred perfectly on frame_center_x and frame_center_y.
        #
        # treatment_top_left = (..., ...)
        # treatment_bottom_right = (..., ...)

        # TODO: Draw the HUD
        # Use cv2.rectangle to draw your treatment zone box on 'cv_image' in blue.
        # Syntax: cv2.rectangle(image, pt1, pt2, color_bgr, thickness)
        #
        # cv2.rectangle(...)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        spray_trigger = False

        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            if area > 500 and perimeter > 0:
                circularity = (4 * math.pi * area) / (perimeter * perimeter)

                if circularity > 0.8:
                    # We found a valid weed!
                    cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)

                    # TODO: Find the centre of the weed
                    # Use cv2.boundingRect(contour) to get x, y, width, height.
                    # Then calculate the centre pixel (weed_center_x, weed_center_y).
                    #
                    # x, y, w, h = ...
                    # weed_center_x = ...
                    # weed_center_y = ...

                    # TODO: Draw a marker on the weed
                    # Use cv2.circle to draw a small dot at the weed's centre pixel.
                    #
                    # cv2.circle(...)

                    # TODO: The Decision Logic
                    # Check if the weed's centre is INSIDE your treatment zone box.
                    # If it is inside: 
                    #    1. Set spray_trigger = True
                    #    2. self.get_logger().warn("WEED CENTRED. SPRAYING!")
                    # If it is outside:
                    #    1. Do NOT trigger the spray.
                    #    2. self.get_logger().info(f"Weed detected at ({weed_center_x}, {weed_center_y}). Navigating...")
                    #
                    # if ...

        # Publish the trigger and the debug HUD
        spray_msg = Bool()
        spray_msg.data = spray_trigger
        self.pub_spray.publish(spray_msg)
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionSpray()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SAFETY: Ensure spray is turned off when shutting down
        stop_msg = Bool()
        stop_msg.data = False
        node.pub_spray.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()