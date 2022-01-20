#!/usr/bin/env python
#
# Support file for node: vision_processing
# 


import cv2
import numpy as np

from RUN_D3D import runD3D_mazeLocators, runD3D_maurerFilter
import os
import subprocess
import json

from cv_bridge import CvBridge, CvBridgeError
import rospy
import rospkg






def VISION_PROCESSOR(data_color, data_depth):
    """

    """

    # Reference Variables
    rospack  = rospkg.RosPack()
    dir_wksp = os.path.join(rospack.get_path('maze_runner'), 'mzrun_ws')


    # Process Input Data from ROS message to .Mat object
    bridge = CvBridge()
    image_color = bridge.imgmsg_to_cv2(data_color, "bgr8")
    image_depth = bridge.imgmsg_to_cv2(data_depth, "passthrough") 

    fpath_color = os.path.join(dir_wksp,'color.tiff')
    fpath_depth = os.path.join(dir_wksp,'depth.tiff')

    cv2.imwrite(fpath_color, image_color)
    cv2.imwrite(fpath_depth, image_depth)


    # Run Pose Processor
    dot_names = ['redDot', 'greenDot', 'blueDot']
    fpath_masked_maze, fpaths_dot_feature = runD3D_mazeLocators(fpath_color, dot_names)


    # Post Process Results
    # input_maze_image_filepath = D3DpostProcessor(fpath_masked_maze, fpaths_dot_feature)


    # Run Mauer-based Path Solver
    # raw_filter_result       = runD3D_maurerFilter(input_maze_image_filepath)
    # maurer_image_filepath   = cleanMaurerPath(raw_filter_result)
    # solved_path, solved_image_filepath = callPathSolver(maurer_image_filepath)


    # Create Message & Return


