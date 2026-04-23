#!/usr/bin/env python3
"""
=============================================================================
  COLOUR CALIBRATOR — HSV Range Tuner for Lab 6
  AgTech ROS Course | PHYS-4000 / AG-3000
=============================================================================

  Interactive tool for finding HSV thresholds to detect red weeds.
  Adjust the sliders until your mask shows ONLY the red balls.

  Usage:
    python3 colour_calibrator.py

  Controls:
    p  = print current HSV values (copy-paste into weed_detector.py)
    q  = quit
"""
import cv2
import numpy as np


def nothing(x):
    pass


# Initialize Webcam (0 = Default)
# If using a USB camera passed through to Docker, this index might need to be 1 or 2
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cv2.namedWindow('Colour Tuner')

# Sliders for HSV
# NOTE: Default H range is wide (0-179) so you can explore before narrowing in.
# Remember that RED wraps around: true red is at BOTH ends of the hue wheel
# (0-10 AND 170-179). See Lab 6 doc for the two-mask workaround.
cv2.createTrackbar('Low H', 'Colour Tuner', 0, 179, nothing)
cv2.createTrackbar('Low S', 'Colour Tuner', 100, 255, nothing)
cv2.createTrackbar('Low V', 'Colour Tuner', 100, 255, nothing)
cv2.createTrackbar('High H', 'Colour Tuner', 179, 179, nothing)
cv2.createTrackbar('High S', 'Colour Tuner', 255, 255, nothing)
cv2.createTrackbar('High V', 'Colour Tuner', 255, 255, nothing)

print("=" * 60)
print("Colour Calibrator running.")
print("  p = print current HSV values (copy into weed_detector.py)")
print("  q = quit")
print("=" * 60)

while True:
    ret, frame = cap.read()

    # If the frame is empty, print why but don't crash immediately
    if not ret:
        print("Failed to grab frame (retrying...)")
        continue

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos('Low H', 'Colour Tuner')
    l_s = cv2.getTrackbarPos('Low S', 'Colour Tuner')
    l_v = cv2.getTrackbarPos('Low V', 'Colour Tuner')
    u_h = cv2.getTrackbarPos('High H', 'Colour Tuner')
    u_s = cv2.getTrackbarPos('High S', 'Colour Tuner')
    u_v = cv2.getTrackbarPos('High V', 'Colour Tuner')

    lower_bound = np.array([l_h, l_s, l_v])
    upper_bound = np.array([u_h, u_s, u_v])

    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    cv2.imshow('Camera Feed', frame)
    cv2.imshow('Mask', mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('p'):
        # Print the current HSV values as Python code you can paste
        # directly into weed_detector.py
        print()
        print("--- Copy these lines into weed_detector.py ---")
        print(f"lower_red = np.array([{l_h}, {l_s}, {l_v}])")
        print(f"upper_red = np.array([{u_h}, {u_s}, {u_v}])")
        print()

cap.release()
cv2.destroyAllWindows()