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
    rospack       = rospkg.RosPack()
    workspace_dir = os.path.join(rospack.get_path('maze_runner'), 'mzrun_ws')


    # Process Input Data from ROS message to .Mat object
    bridge = CvBridge()
    image_color = bridge.imgmsg_to_cv2(data_color, "bgr8")
    image_depth = bridge.imgmsg_to_cv2(data_depth, "passthrough") 

    image_path_color = os.path.join(workspace_dir,'color.tiff')
    image_path_depth = os.path.join(workspace_dir,'depth.tiff')

    cv2.imwrite(image_path_color, image_color)
    cv2.imwrite(image_path_depth, image_depth)

    # Testing
    # image_path_color = os.path.join(workspace_dir,'maze_1.png')
    # cv2.imshow("color", data_color)
    # cv2.imshow("depth", data_depth)
    # cv2.waitKey()


    # Setup Pose Processor
    dot_names = ['redDot', 'greenDot', 'blueDot'] #order MUST be red, green, blue. But exact name may be changed.

    path_redDot     = os.path.join(workspace_dir, 'feature_' + dot_names[0] + '.csv')
    path_greenDot   = os.path.join(workspace_dir, 'feature_' + dot_names[1] + '.csv')
    path_blueDot    = os.path.join(workspace_dir, 'feature_' + dot_names[2] + '.csv')
    path_maze       = os.path.join(workspace_dir, 'mask_MazeOnly.tiff')

    dot_feature_paths = [path_redDot, path_greenDot, path_blueDot]


    # Run Pose Processor
    runD3D_mazeLocators(image_path_color, dot_feature_paths, path_maze)



    # Post Process Results




    # Run Mauer-based Path Solver
    # raw_filter_result       = runD3D_maurerFilter(input_maze_image_filepath)
    # maurer_image_filepath   = cleanMaurerPath(raw_filter_result)
    # solved_path, solved_image_filepath = callPathSolver(maurer_image_filepath)



    # Create Message & Return


