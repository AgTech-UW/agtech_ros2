#!/usr/bin/env python3
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
cv2.createTrackbar('Low H', 'Colour Tuner', 0, 179, nothing)
cv2.createTrackbar('Low S', 'Colour Tuner', 100, 255, nothing)
cv2.createTrackbar('Low V', 'Colour Tuner', 100, 255, nothing)
cv2.createTrackbar('High H', 'Colour Tuner', 10, 179, nothing)
cv2.createTrackbar('High S', 'Colour Tuner', 255, 255, nothing)
cv2.createTrackbar('High V', 'Colour Tuner', 255, 255, nothing)

while True:
    ret, frame = cap.read()
    
    # If the frame is empty, print why but don't crash immediately
    if not ret:
        print("Failed to grab frame (Retrying...)")
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

    cv2.imshow('Colour Tuner', frame)
    cv2.imshow('Mask', mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()