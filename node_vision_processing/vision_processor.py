#!/usr/bin/env python
#
# Support file for node: vision_processing
# 


import cv2
import numpy as np

from tools_dream3d import runD3D_mazeLocators, runD3D_mazePath
import os
import subprocess
import json

import rospy
import rospkg
import time



def main():
    # Reference Variables
    rospack       = rospkg.RosPack()
    workspace_dir = os.path.join(rospack.get_path('maze_runner'), 'mzrun_ws')

    image_path_color = os.path.join(workspace_dir,'maze_1.png')


    # Testing
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
    start = time.time()
    runD3D_mazeLocators(image_path_color, dot_feature_paths, path_maze)



    rospy.loginfo("PostProcess Runner took: " + str(time.time() - start) + " seconds.")



if __name__ == '__main__':
    main()
