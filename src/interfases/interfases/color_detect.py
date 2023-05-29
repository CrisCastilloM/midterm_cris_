#!/usr/bin/env python3
import cv2 
import numpy as np 
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Int32MultiArray

import ctypes
from ctypes.util import find_library
lib = ctypes.cdll.LoadLibrary(find_library("multiplier"))
multiply100 = lib.multiplicar_por_100

rclpy.init()
node = Node("color_detect_node")
pub = node.create_publisher(Int32MultiArray, '/coordinates', 10)

def publish_to_ros(coordinate):
  print('Publishing coordinates:', coordinate)
  msg = Int32MultiArray()
  msg.data = coordinate
  pub.publish(msg)


def run():
  cap = cv2.VideoCapture(0)
  
  while True:
    temp, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    low = np.array([86, 191, 118])
    high = np.array([133, 255, 255])

    filtered = cv2.inRange(hsv, low, high)
    contours, temp = cv2.findContours(
      filtered, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cont in contours:
      aproximation = cv2.approxPolyDP(cont,
        0.02 * cv2.arcLength(cont, True), True)
      contour_area = cv2.contourArea(cont)
      if contour_area > 400:
        cv2.drawContours(frame, [cont], 0, (0,0,0), 5)

        if len(aproximation) == 4:
            M = cv2.moments(cont)
            if M['m00'] != 0:
              cy = int(M['m01'] / M['m00'])
              cx = int(M['m10'] / M['m00'])
              coordinate = [multiply100(cx), multiply100(cy)]
              publish_to_ros(coordinate)
              cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
              cv2.putText(frame, "CENTER", (cx - 20, cy - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
              cv2.drawContours(frame, [cont], -1, (0, 255, 0), 2)

    cv2.imshow("video", frame)
    cv2.imshow("color filter", filtered)
    if (cv2.waitKey(30) == 27):
      break
    
  try:
    rclpy.spin(node)
  finally:
    cv2.destroyAllWindows()
    cap.release()
    node.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  run()