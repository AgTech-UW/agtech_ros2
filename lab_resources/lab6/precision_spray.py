#!/usr/bin/env python3
"""
Lab 6, Part 7: Precision Spraying.

This node uses computer vision to find a red marker, calculates its
centre, and triggers a spray signal ONLY if the weed is within a
specific "treatment zone" in the centre of the camera frame.

Subscribes:
    /image_raw              sensor_msgs/Image

Publishes:
    /weed_detector/debug    sensor_msgs/Image
    /weed_detector/mask     sensor_msgs/Image
    /tractor/spray          std_msgs/Bool

YOUR JOB:
Fill in every line marked TODO. Focus on the spatial logic and
OpenCV drawing functions.
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
            Image, '/image_raw', self.image_callback, 10
        )

        self.pub_debug = self.create_publisher(
            Image, '/weed_detector/debug', 10
        )

        self.pub_mask = self.create_publisher(
            Image, '/weed_detector/mask', 10
        )

        self.pub_spray = self.create_publisher(
            Bool, '/tractor/spray', 10
        )

        self.bridge = CvBridge()

        self.get_logger().info("Precision Spray ready. Looking for targets...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Convert the ROS Image message into an OpenCV image.
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion failed: {e}")
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # Convert the image to HSV colour space for easier colour detection.

        # ------------------------------------------------------------
        # 1. Create the red mask
        # ------------------------------------------------------------
        # TODO: Replace these default values with your calibrated values
        # from colour_calibrator.py.

        # Range 1: red near H = 0.
        lower_red1 = np.array([0, 140, 51])
        upper_red1 = np.array([10, 255, 255])

        # Range 2: red near H = 179.
        lower_red2 = np.array([170, 140, 51])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Clean up small white speckles before contour detection.
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Publish the mask for debugging in rqt_image_view.
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8")) # Publish the mask so it can be viewed in rqt_image_view.

        # ------------------------------------------------------------
        # 2. Find the centre of the camera frame
        # ------------------------------------------------------------
        # Hint: cv_image.shape returns (height, width, channels)

        # TODO:
        # frame_height, frame_width, channels = ...
        # frame_center_x = ...
        # frame_center_y = ...

        # ------------------------------------------------------------
        # 3. Define the 100 by 100 pixel treatment zone
        # ------------------------------------------------------------
        treatment_size = 100
        half_zone = int(treatment_size / 2)

        # TODO:
        # treatment_top_left = (..., ...)
        # treatment_bottom_right = (..., ...)

        # ------------------------------------------------------------
        # 4. Draw the HUD treatment zone
        # ------------------------------------------------------------
        # Draw the treatment zone box in blue.
        # OpenCV colours use BGR, so blue is (255, 0, 0).

        # TODO:
        # cv2.rectangle(...)

        # Optional centre crosshair for readability.
        #cv2.circle(...)

        # ------------------------------------------------------------
        # 5. Find contours
        # ------------------------------------------------------------
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        spray_trigger = False

        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            # Filter 1: ignore tiny blobs.
            if area > 500 and perimeter > 0:

                # Filter 2: check circularity.
                circularity = (4 * math.pi * area) / (perimeter * perimeter)

                if circularity > 0.7:
                    # We found a valid weed marker.
                    cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)

                    # ------------------------------------------------------------
                    # 6. Find the centre of the weed
                    # ------------------------------------------------------------
                    # Use cv2.boundingRect(contour) to get x, y, width, height.
                    # Then calculate the centre pixel.

                    # TODO:
                    # x, y, w, h = ...
                    # weed_center_x = ...
                    # weed_center_y = ...

                    # ------------------------------------------------------------
                    # 7. Draw a marker on the weed
                    # ------------------------------------------------------------
                    # Draw a small red dot at the weed centre.
                    # OpenCV colours use BGR, so red is (0, 0, 255).

                    # TODO:
                    # cv2.circle(...)

                    # ------------------------------------------------------------
                    # 8. Decision logic
                    # ------------------------------------------------------------
                    # Check if the weed centre is inside the treatment zone.
                    #
                    # If inside:
                    #   1. Set spray_trigger = True
                    #   2. Log: "WEED CENTRED. SPRAYING!"
                    #
                    # If outside:
                    #   1. Do not trigger spray
                    #   2. Log: "Weed detected at (X, Y). Navigating..."

                    # TODO:
                    # inside_x = ...
                    # inside_y = ...
                    #
                    # if ...:
                    #     ...
                    # else:
                    #     ...

        # ------------------------------------------------------------
        # 9. Publish the trigger and the debug HUD
        # ------------------------------------------------------------
        spray_msg = Bool()
        spray_msg.data = spray_trigger
        self.pub_spray.publish(spray_msg) # Publish the spray trigger signal to the tractor.

        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8")) # Publish the debug HUD image with contours and markers drawn on it.


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionSpray()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety: ensure spray is turned off when shutting down.
        stop_msg = Bool()
        stop_msg.data = False
        node.pub_spray.publish(stop_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()