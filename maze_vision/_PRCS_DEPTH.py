#!/usr/bin/env python3
#
# Support file for node: vision_processing
# 

import cv2
import numpy as np


###################
## Class Methods ##
###################

def smooth_depth_map(self, depth_map):
    """
        Kernel sizes are hardcoded to 1270x720 input depth map.
        Future todo for variable kernel size based on input image size.
    """
    import cv2

    # Otsu Thresholding
    # rescaling as CV2 threshold requires uint8
    # dMax = np.max(depth_map)
    # img_scaled = cv2.normalize(img_gauss, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX).astype('uint8')
    # ret3, img_otsu = cv2.threshold(img_scaled, 0, 255, cv2.THRESH_OTSU)
    # cv2.imshow('image', img_otsu*18)
    # cv2.imwrite('mzrun_ws/dmap_otsu.tiff', img_otsu)

    # Region Simplifyication
    kern = np.ones((10,10), np.uint16)
    img_pro = cv2.dilate(depth_map, kern, iterations=3)
    # img_pro = cv2.morphologyEx(img_gauss, cv2.MORPH_CLOSE, kernel)

    # Region Averaging
    img_blur = cv2.blur(img_pro, ksize=(25,25))

    # Debug
    # test = cv2.normalize(img_blur, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX).astype('uint8')
    # cv2.imshow('test', test)

    cv2.imwrite('mzrun_ws/dmap_blurred.tiff', img_blur)
    # cv2.imwrite('mzrun_ws/dmap_gauss.tiff', img_gauss)
    
    # cv2.waitKey(0)

    return img_blur
