#!/usr/bin/env python3
"""
Lab 6: Weed Detector - red ball detection using HSV + circularity.

Subscribes:
    /image_raw              sensor_msgs/Image  - Camera feed

Publishes:
    /weed_detector/debug    sensor_msgs/Image  - Frame with weeds outlined
    /weed_detector/mask     sensor_msgs/Image  - Black/white HSV mask
    /tractor/spray          std_msgs/Bool      - True when weed detected
"""

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

        # 1. Subscribe to the raw camera feed.
        # The usb_cam driver publishes to /image_raw by default.
        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )

        # 2. Publishers.
        # /weed_detector/debug shows the camera image with green outlines.
        self.pub_debug = self.create_publisher(
            Image, '/weed_detector/debug', 10
        )

        # /weed_detector/mask shows the black/white HSV mask.
        # White pixels are pixels the detector thinks are red.
        self.pub_mask = self.create_publisher(
            Image, '/weed_detector/mask', 10
        )

        # /tractor/spray publishes True when a weed marker is detected.
        self.pub_spray = self.create_publisher(
            Bool, '/tractor/spray', 10
        )

        self.bridge = CvBridge()
        self.get_logger().info("Weed detector active. Waiting for images...")

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message into an OpenCV image.
            # "bgr8" is OpenCV's normal colour format.
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            # Log the error so students can see why the callback failed.
            self.get_logger().error(f"CvBridge conversion failed: {e}")
            return

        # 3. Convert from BGR to HSV.
        # HSV makes colour filtering easier because Hue represents the colour,
        # while Saturation and Value represent colour intensity and brightness.
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 4. Create the red mask.
        # Red wraps around the HSV hue wheel in OpenCV.
        # This means red appears near H = 0 and also near H = 179.
        # Therefore, we create two masks and combine them.

        # Range 1: lower end of red hue range.
        # Paste the first range printed by colour_calibrator.py here.
        # TODO: Replace these default values with the values printed by colour_calibrator.py.

        #lower_red1 = np.array([0, 140, 51])
        #upper_red1 = np.array([10, 255, 255])

        #lower_red2 = np.array([170, 140, 51])
        #upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        # Combine both red masks into one final mask.
        mask = cv2.bitwise_or(mask1, mask2)

        # 5. Clean up the mask before finding contours.
        # Small reflections, camera noise, and compression artifacts can create
        # tiny white speckles. Erode removes small white dots, and dilate grows
        # the remaining white regions back to roughly their original size.
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Publish the mask so it can be viewed in rqt_image_view.
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))

        # 6. Find contours.
        # A contour is the outline of a white blob in the mask.
        # RETR_EXTERNAL keeps only the outermost contours, which is enough
        # for this lab because we only care about separate red blobs.
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        found_weed = False

        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            # Filter 1: Size.
            # Ignore tiny blobs under 500 pixels. These are usually noise,
            # reflections, or small red objects in the background.
            if area > 500 and perimeter > 0:

                # Filter 2: Shape.
                # Circularity compares the blob's area to its perimeter.
                #
                # Formula:
                # circularity = 4 * pi * Area / Perimeter^2
                #
                # A perfect circle has circularity = 1.0.
                # In real camera images, a red ball may score lower because of
                # shadows, pixelated edges, blur, or imperfect masking.
                circularity = (4 * math.pi * area) / (perimeter * perimeter)

                # Since our markers are red balls, their 2D silhouette should
                # be roughly circular. A threshold of 0.7 is forgiving enough
                # for classroom lighting and webcam noise.
                if circularity > 0.7:
                    found_weed = True

                    # Draw a green outline around the detected weed marker.
                    cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 3)

        # 7. Publish spray trigger.
        # True means a red circular marker was found in this frame.
        spray_msg = Bool()
        spray_msg.data = found_weed
        self.pub_spray.publish(spray_msg)

        # 8. Publish debug image.
        # This image is the original camera feed with green outlines drawn on it.
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = WeedDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety shutdown:
        # Publish one final False so the sprayer does not remain active
        # after the node is stopped.
        stop_msg = Bool()
        stop_msg.data = False
        node.pub_spray.publish(stop_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()