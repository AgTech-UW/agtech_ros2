#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class WeedDetector(Node):
    def __init__(self):
        super().__init__('weed_detector')
        
        # 1. Subscribe to the raw camera feed
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        # 2. Publishers (Debug View & Spray Trigger)
        self.pub_debug = self.create_publisher(Image, '/weed_detector/debug', 10)
        self.pub_spray = self.create_publisher(Bool, '/tractor/spray', 10)
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        # 3. CONVERT TO HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 4. INSERT YOUR CALIBRATED NUMBERS HERE
        # TODO: Replace underscores with your numbers from Part 3
        lower_red = np.array([__, ___, ___])   
        upper_red = np.array([__, ___, ___])
        
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # 5. FIND CONTOURS (Blobs)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        found_weed = False
        
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            
            # Filter 1: Size (Ignore tiny specks < 500 pixels)
            if area > 500 and perimeter > 0:
                
                # Filter 2: Shape (Circularity)
                # Formula: 4 * PI * Area / Perimeter^2
                circularity = (4 * math.pi * area) / (perimeter * perimeter)
                
                # Perfect circle is 1.0. 
                # Since the camera is angled, we accept > 0.6 (Squashed Ellipse)
                if circularity > 0.6:
                    found_weed = True
                    # Draw Green Outline around the weed
                    cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 3)

        # 6. PUBLISH TRIGGER
        spray_msg = Bool()
        spray_msg.data = found_weed
        self.pub_spray.publish(spray_msg)

        # 7. PUBLISH DEBUG IMAGE
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WeedDetector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()