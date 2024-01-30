#!/bin/python3
import cv2
# Normal HSV = (41.7, 100, 41.2)[GIMP] = (20.7, 255, 105.06)
image = cv2.imread("screenshot.png", cv2.IMREAD_COLOR)
image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
image = cv2.inRange(image, (6, 80, 80), (50, 255, 255))
cv2.imshow("binary image", image)
cv2.waitKey(0)