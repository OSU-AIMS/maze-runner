#!/usr/bin/env python
#
# Support file for node: vision_processing
# 


import cv2
import numpy as np



# testing
bridge = CvBridge()
imgc = bridge.imgmsg_to_cv2(self.color, "bgr8")
imgg = bridge.imgmsg_to_cv2(self.depth, "passthrough") 

cv2.imshow("color", imgc)
cv2.imshow("depth", imgg)
cv2.waitKey()

