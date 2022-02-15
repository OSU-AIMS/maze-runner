#!/usr/bin/env python
#
# Support file for node: vision_processing
# 



from RUN_D3D import runD3D_mazeLocators, runD3D_maurerFilter
from PRCS_PATH_SOLVER import cleanMaurerPath, callPathSolver
from PRCS_D3D_FEATURES import CONTEXTUALIZE_D3D_FEATURES

import cv2
from cv_bridge import CvBridge, CvBridgeError

import os
import rospy
import rospkg

import csv
import os
import numpy as np





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
    maze_size = [0.18, 0.18] # Realworld Measurement (in meters)
    prcs_features = CONTEXTUALIZE_D3D_FEATURES(dot_names, fpaths_dot_feature, fpath_masked_maze, maze_size)
    input_maze_image_filepath = prcs_features.exportResults()


    # Run Mauer-based Path Solver
    raw_filter_result       = runD3D_maurerFilter(input_maze_image_filepath)
    maurer_image_filepath   = cleanMaurerPath(raw_filter_result)

    # Run Python Maze Solver
    solved_path, solved_image_filepath = callPathSolver(maurer_image_filepath)


    # Create Message & Return


