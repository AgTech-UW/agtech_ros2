#!/usr/bin/env python3
"""
Lab 6: Weed Detector - red ball detection (HSV + circularity).
 
Subscribes: /image_raw              (sensor_msgs/Image)  - Camera feed
Publishes:  /weed_detector/debug    (sensor_msgs/Image)  - Frame with weeds outlined
            /tractor/spray          (std_msgs/Bool)      - True when weed detected
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

        # 1. Subscribe to the raw camera feed
        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )

        # 2. Publishers (debug view & spray trigger)
        self.pub_debug = self.create_publisher(
            Image, '/weed_detector/debug', 10
        )
        self.pub_spray = self.create_publisher(
            Bool, '/tractor/spray', 10
        )

        self.bridge = CvBridge()
        self.get_logger().info("Weed detector active. Waiting for images...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            # Log the error so we can see WHY the callback failed.
            # Silent failures will make students think the node is working
            # when it's actually broken.
            self.get_logger().error(f"CvBridge conversion failed: {e}")
            return

        # 3. CONVERT TO HSV
        # Note: OpenCV is BGR-native (not RGB!), so we use COLOR_BGR2HSV.
        # Many online tutorials show COLOR_RGB2HSV -- those are for libraries
        # like Pillow or matplotlib. Using the wrong one swaps your red and
        # blue channels and your detector silently looks for blue instead.
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 4. INSERT YOUR CALIBRATED NUMBERS HERE
        # TODO: Run colour_calibrator.py, tune the sliders until your red
        # balls show up cleanly in the mask, then press 'p' in that window
        # and paste the output here.
        lower_red = np.array([__, __, __])
        upper_red = np.array([__, __, __])

        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # 5. FIND CONTOURS (blobs)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        found_weed = False
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            # Filter 1: Size (Ignore tiny specks < 500 pixels)
            # 500 px ~= a 22x22 blob. Tune UP if you're far from the marker
            # (the ball will look small in the frame); tune DOWN if you're
            # close. Without this filter, JPEG compression artifacts and
            # small reflections trigger false detections every frame.
            if area > 500 and perimeter > 0:

                # Filter 2: Shape (circularity)
                # Formula: 4 * pi * Area / Perimeter^2
                # Perfect circle = 1.0.
                #
                # Weeds are red SPHERES (not flat stickers or rings).
                # Thanks to point symmetry, a sphere's silhouette is always
                # a circle regardless of camera angle. So we can use a tight
                # threshold (0.8) to reject rectangles, rings, and irregular
                # blobs while still accepting real red balls.
                circularity = (4 * math.pi * area) / (perimeter * perimeter)

                if circularity > 0.8:
                    found_weed = True
                    # Draw green outline around the detected weed
                    cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 3)

        # 6. PUBLISH SPRAY TRIGGER
        spray_msg = Bool()
        spray_msg.data = found_weed
        self.pub_spray.publish(spray_msg)

        # 7. PUBLISH DEBUG IMAGE
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = WeedDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SAFETY: explicitly tell the sprayer to shut off on shutdown.
        # Without this, the last /tractor/spray value could latch as True,
        # leaving a real sprayer solenoid stuck open.
        stop_msg = Bool()
        stop_msg.data = False
        node.pub_spray.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()