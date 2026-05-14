#!/usr/bin/env python3
"""
Lab 6: Colour Calibrator - interactive HSV threshold tuner.

Controls:
  p  - print current HSV values for TWO red masks
  q  - quit

This calibrator is designed for red objects.
Red wraps around the OpenCV HSV hue wheel:
  - red near H = 0
  - red near H = 179

So the final detector uses two masks and combines them.
"""

import cv2
import numpy as np


def nothing(x):
    pass


# Initialize Webcam
cap = cv2.VideoCapture(0)

# Match the ROS usb_cam settings as closely as possible
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# These may or may not work depending on camera/driver, but are safe to try
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)

cv2.namedWindow('Colour Tuner')

# Instead of tuning Hue from 0 to 179, we keep the red Hue ranges fixed.
# Students tune Saturation and Value.
cv2.createTrackbar('Low S', 'Colour Tuner', 100, 255, nothing)
cv2.createTrackbar('Low V', 'Colour Tuner', 50, 255, nothing)
cv2.createTrackbar('High S', 'Colour Tuner', 255, 255, nothing)
cv2.createTrackbar('High V', 'Colour Tuner', 255, 255, nothing)

print("=" * 60)
print("Colour Calibrator running.")
print("This calibrator uses TWO red masks:")
print("  Mask 1: H = 0 to 10")
print("  Mask 2: H = 170 to 179")
print()
print("Tune S and V until the red ball is white and the background is black.")
print("  p = print copy-paste code for weed_detector.py")
print("  q = quit")
print("=" * 60)

while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame (retrying...)")
        continue

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Read slider values
    l_s = cv2.getTrackbarPos('Low S', 'Colour Tuner')
    l_v = cv2.getTrackbarPos('Low V', 'Colour Tuner')
    u_s = cv2.getTrackbarPos('High S', 'Colour Tuner')
    u_v = cv2.getTrackbarPos('High V', 'Colour Tuner')

    # Fixed red hue ranges
    lower_red1 = np.array([0, l_s, l_v])
    upper_red1 = np.array([10, u_s, u_v])

    lower_red2 = np.array([170, l_s, l_v])
    upper_red2 = np.array([179, u_s, u_v])

    # Create two masks and combine them
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    cv2.imshow('Camera Feed', frame)
    cv2.imshow('Mask', mask)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    elif key == ord('p'):
        print()
        print("--- Copy these lines into weed_detector.py ---")
        print()
        print("# Red wraps around the HSV hue wheel, so we use two masks.")
        print(f"lower_red1 = np.array([0, {l_s}, {l_v}])")
        print(f"upper_red1 = np.array([10, {u_s}, {u_v}])")
        print()
        print(f"lower_red2 = np.array([170, {l_s}, {l_v}])")
        print(f"upper_red2 = np.array([179, {u_s}, {u_v}])")
        print()
        print("mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)")
        print("mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)")
        print("mask = cv2.bitwise_or(mask1, mask2)")
        print()

cap.release()
cv2.destroyAllWindows()