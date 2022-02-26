#!/usr/bin/env python
#
# Support file for node: vision_processing
# 



from RUN_D3D import runD3D_mazeLocators, runD3D_maurerFilter
from PRCS_PATH_SOLVER import cleanMaurerPath, callPathSolver
from PRCS_D3D_FEATURES import CONTEXTUALIZE_D3D_FEATURES

import cv2

import os
import rospy
import rospkg

import csv
import os
import numpy as np

import tf




def VISION_PROCESSOR(image_color, image_depth):
    """

    """

    # Reference Variables
    rospack  = rospkg.RosPack()
    dir_wksp = os.path.join(rospack.get_path('maze_runner'), 'mzrun_ws')

    fpath_color = os.path.join(dir_wksp,'color.tiff')
    fpath_depth = os.path.join(dir_wksp,'depth.tiff')

    cv2.imwrite(fpath_color, image_color)
    cv2.imwrite(fpath_depth, image_depth)


    # Run Pose Processor
    dot_names = ['redDot', 'greenDot', 'blueDot']
    fpath_masked_maze, fpaths_dot_feature = runD3D_mazeLocators(fpath_color, dot_names)


    # Post Process Results
    maze_size = [0.18, 0.18] # Realworld Measurement (in meters)
    features = CONTEXTUALIZE_D3D_FEATURES(dot_names, fpaths_dot_feature, fpath_masked_maze, maze_size)
    input_maze_image_filepath = features.exportResults()

    rot3d_tf = np.identity(4)
    rot3d_tf[:-1,:-1] = features.rotMatrix.astype()
    rot3d_q = tf.transformations.quaternion_from_matrix(rot3d_tf)


    # Simplify & Clean Cropped Maze Image
    raw_filter_result       = runD3D_maurerFilter(input_maze_image_filepath)
    maurer_image_filepath   = cleanMaurerPath(raw_filter_result)


    # Run Python Maze Solver on Cleaned Image
    solved_csv_filepath, solved_image_filepath = callPathSolver(maurer_image_filepath)


    # Depth
    # temporary, grab depth at origin and assume flat.
    # TODO: new method to get depth at every waypoing along path
    


    # Calculate Pose
    # maze_pose_relative_2_camera = [x,y,z,x,y,z,w]

    # Return Results
    # projected_2d_pose = [x,y,theta]
    result = [features.scale, projected_2d_pose, maze_pose_relative_2_camera, path]

    return result


