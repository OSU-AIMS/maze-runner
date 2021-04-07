#!/usr/bin/env python

#####################################################
##   Maze Runner: Move Robotic Arm Through Maze    ##
##                                                 ##
##   Capabilities Demonstrated                     ##
##   * Planar Maze Following                       ##
##                                                 ##
#####################################################

# Software License Agreement (Apache 2.0 License)
#
# Copyright (c) 2021, The Ohio State University
# Center for Design and Manufacturing Excellence (CDME)
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
# All rights reserved.
#
# Author: Adam Buynak

#####################################################

import os
import numpy as np

from process_path import prepare_path_transforms_list
from process_path import prepare_path_tf_ready

from transformations import transformations

#import robot_support
from robot_support import all_close
from robot_support import moveManipulator


## From Robot Support
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from motoman_msgs.srv import ReadSingleIO, WriteSingleIO

## Quaternion Tools
from tf.transformations import euler_from_quaternion, quaternion_from_euler

## Maze Runner Specific
import csv
import json
from rospy_message_converter import json_message_converter
from d3d_post_process import Dream3DPostProcess

#####################################################
## Import Realsense Control Tools
from realsense_vision_tools import REALSENSE_VISION, timer_wait

# Used for Advanced Mode Setup & Point Cloud Streaming
import pyrealsense2 as rs
import time
#import sys

# Used for Depth Filtering
#import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures

# Color Image processing
import cv2

#####################################################


def retrieve_pose_from_dream3d(workspace_path, image_path, locator_scalar_tolerance=1000):
    """
    Input Dream3D. Don't know what this looks like. 
    Place holder! Yay!
    """
    import os
    import subprocess
    import json

    pipeline = os.path.dirname(workspace_path) +'/dream3d_pipelines/filter_irs_image.json'

    path_redDot   = workspace_path + '/feature_redDot.csv'
    path_greenDot = workspace_path + '/feature_greenDot.csv'
    path_blueDot  = workspace_path + '/feature_blueDot.csv'

    # Setup & Run Dream3D / Simple Pipeline
    with open(pipeline, 'r') as jsonFile:
        data = json.load(jsonFile)

    data["00"]["FileName"] = image_path

    # Feature Segmenter Settings
    data["14"]["ScalarTolerance"] = locator_scalar_tolerance
    data["15"]["ScalarTolerance"] = locator_scalar_tolerance
    data["16"]["ScalarTolerance"] = locator_scalar_tolerance

    data["23"]["FeatureDataFile"] = path_redDot
    data["24"]["FeatureDataFile"] = path_greenDot
    data["25"]["FeatureDataFile"] = path_blueDot

    data["26"]["OutputFilePath"] = workspace_path + '/feature_redDot_array.csv'
    data["28"]["OutputFilePath"] = workspace_path + '/filter_irs_image.dream3d'

    data["31"]["FileName"] = workspace_path + '/mask_MazeOnly.tiff'
    data["34"]["FileName"] = workspace_path + '/mask_mostlyBlue.tiff'

    with open(pipeline, 'w') as jsonFile:
        json.dump(data, jsonFile)

    # Workaround to supress output.
    d3d_output = open(workspace_path + '/temp_pipelineResult.txt', 'a')
    subprocess.call(["/opt/dream3d/bin/PipelineRunner", "-p", pipeline], stdout=d3d_output, stderr=d3d_output)

    featureData_dots_filepaths = [path_redDot, path_greenDot, path_blueDot]

    print(">> Dream3D Pipeline Runner Complete")
    return featureData_dots_filepaths




def characterize_maze(camera, workspace_path, img_id, maze_size, featureData_dots_filepaths):


    # Post Process Features to find Rotation, mazeCentroid, mazeScale
    color_names = ["red", "green", "blue"]
    d3d = Dream3DPostProcess(color_names, featureData_dots_filepaths, maze_size, tolerance=80)

    rotationMatrix = d3d.rotMatrix
    mazeCentroid = d3d.mazeCentroid
    mazeScale = d3d.scale


    # Apply Depth Information
    depth_array = np.load(str(workspace_path) + '/' + str(img_id) + '_depth.npy')
    
    for color in color_names:

        # Raw Value Pulled from Depth. Not corrected for length
        # new_depth = depth_array[d3d.dots[color]['centroid'][0], d3d.dots[color]['centroid'][1]]
        # print('Depth @ Point', color)
        # print(new_depth)

        # Corrected Depth Value provided by iRS API.
        # todo: find cleaner approach to use this, or use intrincis to fix above calculation.
        pixel_coord = [d3d.dots[color]['centroid'][0], d3d.dots[color]['centroid'][1]]

        correctedDepth = camera.get_depth_at_point(pixel_coord[0], pixel_coord[1])
        print('RS Depth @ Point', color)
        print(correctedDepth)

        d3d.dots[color]['centroid'][2] = correctedDepth
        print(d3d.dots[color]['centroid'])

        # Transform Dots into World Coordinates
        x, y, z = camera.get_3d_coordinate_at_point(pixel_coord, correctedDepth)
        d3d.dots[color]['centroid_camera_frame'] = [0,0,0] #temporary filler
        d3d.dots[color]['centroid_camera_frame'][0] = x
        d3d.dots[color]['centroid_camera_frame'][1] = y
        d3d.dots[color]['centroid_camera_frame'][2] = z 

        print("START DEBUGGING HERE NEXT")
        print("REAL DOT COORDINATES", color)
        print(x,y,z)
        print('')


    mazeOrigin = d3d.dots['green']['centroid']
    print(mazeOrigin)


    print(">> Characterize Maze, Complete.")
    return mazeCentroid, rotationMatrix, mazeScale, mazeOrigin


class DataSaver(object):
    """
    All data saving functions wrapped in one tool.
    """

    #todo: this assumes only ONE robot planning scene. Actually using TWO scenes. DataSaver does NOT make this distinction when saving poses!!!
    def __init__(self, mzrun_ws, robot, camera, maze_size = [0.18, 0.18]):
        #Setup
        self.workspace = mzrun_ws
        self.robot = robot
        self.camera = camera
        self.maze_size = maze_size

        setup_dict = {}
        setup_dict['counter'] = []
        setup_dict['poses'] = []
        setup_dict['images'] = []
        setup_dict['maze_rotationMatrix'] = []
        setup_dict['maze_centroid'] = []
        setup_dict['scale'] = []
        setup_dict['maze_origin'] = []

        self.all_data = setup_dict

    def capture(self, img_counter, find_maze=False):
        #img_path = self.camera.capture_singleFrame_color(self.workspace + '/' + str(img_counter))
        img_path = self.camera.capture_singleFrame_alignedRGBD(self.workspace + '/' + str(img_counter))
        pose = self.robot.lookup_pose()

        if find_maze: 
            # Run Vision Pipeline, find Location & Rotation of Maze
            featureData_dots_filepaths = retrieve_pose_from_dream3d(self.workspace, img_path, 1000)
            centroid, rotationMatrix, scale, mazeOrigin  = characterize_maze(self.camera, self.workspace, img_id=img_counter, maze_size=self.maze_size, featureData_dots_filepaths=featureData_dots_filepaths)

            self.save_data(self.workspace, img_counter, img_path, pose, centroid, rotationMatrix, scale, mazeOrigin)
        else:
            self.save_data(self.workspace, img_counter, img_path, pose, np.array([0]), np.array([0]), 0, np.array([0]))

    def save_data(self, mzrun_ws, img_counter, img_path, pose, maze_centroid, maze_rotationMatrix, scale, mazeOrigin) :
        """
        Take Photo AND Record Current Robot Position
        :param robot:     Robot Instance
        :param camera:    Realsense Camera Instance
        :param mzrun_ws:  Local Workspace Path for Variables
        :param image_counter: Counter Input for consequitive location is saved at. 
        :return           Updated Storage Dicitonary
        """

        add_data = self.all_data

        add_data['counter'].append(img_counter)
        add_data['images'].append(img_path)
        add_data['poses'].append(json_message_converter.convert_ros_message_to_json(pose))
        add_data['maze_centroid'].append(np.ndarray.tolist(maze_centroid))
        add_data['maze_rotationMatrix'].append(np.ndarray.tolist(maze_rotationMatrix))
        add_data['scale'].append(scale)
        add_data['maze_origin'].append(np.ndarray.tolist(mazeOrigin))

        with open(mzrun_ws + '/camera_poses.json', 'w') as outfile:
            json.dump(add_data, outfile, indent=4)

        self.all_data = add_data

    def last_recorded(self):
        # Build Temporary Dictionary to Return latest Data

        latest_data = {
            'counter': self.all_data['counter'][-1],
            'images': self.all_data['images'][-1],
            'poses' : self.all_data['poses'][-1],
            'maze_centroid' : self.all_data['maze_centroid'][-1],
            'maze_rotationMatrix' : self.all_data['maze_rotationMatrix'][-1],
            'scale' : self.all_data['scale'][-1],
            'maze_origin' : self.all_data['maze_origin'][-1],
        }

        return latest_data



#####################################################
def main():

    print("")
    print("----------------------------------------------------------")
    print("           Maze Runner       (TOP LEVEL)                  ")
    print("----------------------------------------------------------")
    print("Example developed by ACBUYNAK. Spring 2021")
    #print("Note: You will be unable to interrupt program if running\n from terminal. Alternatively use IDLE console.")
    #print("Press Enter to advance script when prompted.")
    print("")





    ############################
    #Debug Settings
    motion_testing = True
    camera_testing = True
    path_testing   = False

    try:

        # Setup Robot
        robot_camera = moveManipulator(eef='r2_eef_camera')
        robot_camera.set_accel(0.15)
        robot_camera.set_vel(0.20)
        robot_pointer = moveManipulator(eef='r2_eef_pointer')
        robot_pointer.set_accel(0.05)
        robot_pointer.set_vel(0.05)


        # Setup Camera
        if camera_testing:
            camera = REALSENSE_VISION(set_color=[640,480,30], set_depth=[640,480,30], max_distance=5.0) # Higher Resolution made difficult... (set_color=[1280,720,6], set_depth=[1280,720,6])
        
        # Setup Working Directory
        mzrun_pkg = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        mzrun_ws = mzrun_pkg + '/mzrun_ws'

        try:
            os.makedirs(mzrun_ws, True)
        except OSError:
            if not os.path.isdir(mzrun_ws):
                raise
        os.chmod(mzrun_ws, 0777)

        # Setup Image Record JSON
        if camera_testing:
            database = DataSaver(mzrun_ws, robot_camera, camera)
        img = 0


        ############################
        # Move to Known Start Position: All-Zeros
        if motion_testing:
            raw_input('Go to Maze Overlook Crouch Position <enter>')
            robot_camera.goto_named_target("maze_overlook_crouch")

        # Testing: Increment Robot Down from overloop position
        if camera_testing:

            # Move to Start Position
            start_pose = robot_camera.lookup_pose()
            start_pose.position.z -= 0.7
            if motion_testing:
            raw_input('Go to Maze Overlook Crouch Position <enter>')
            robot_camera.goto_Quant_Orient(start_pose)

            # Record First Camera Set
            timer_wait() #ensure camera has had time to buffer initial frames
            database.capture(img, find_maze=False)

            for img in range(1,2):

                # Move Robot
                if motion_testing:
                    raw_input('incr <enter>')
                    tmp_pose = robot_camera.lookup_pose()
                    tmp_pose.position.z -= 0.3
                    robot_camera.goto_Quant_Orient(tmp_pose)

                # Capture Data
                database.capture(img, find_maze=True)   
                latest_data = database.last_recorded()

                last_mazeOrigin = latest_data['maze_origin']
                last_scale      = latest_data['scale']
                
                print('\nDebug')
                print('last_origin',last_mazeOrigin)
                print('last_scale',last_scale)


        # Path Following Demo:
        if path_testing:

            #maze_closelook_stool = [2.09, -2.4563, 1.037, 0.707, -0.707, 0, 0]
            #robot_camera.goto_Quant_Orient(maze_closelook_stool)

            maze_stool_start = [2.05, -2.47, 0.72, 0.97, 0.242, 0.014, 0.012]
            robot_pointer.goto_Quant_Orient(maze_stool_start)


            # Set Maze to be Solved
            #solved_maze_path = raw_input('Input PreSolved Maze Path: ')
            print("AutoInput Pre-Solved Maze Path:  'lab_demo_soln.csv'")
            solved_maze = '/demo/lab_demo_soln.csv'
            solved_maze_path = mzrun_pkg + solved_maze

            # XY Process Path into Transformation List
            path_as_xyz = prepare_path_transforms_list(solved_maze_path, scaling_factor=0.020)
            path_as_tf_matrices = prepare_path_tf_ready(path_as_xyz)

            # Find & Apply Rigid Body Transform to Maze 
            tf = transformations()
            body_rot = np.matrix('1 0 0; 0 1 0; 0 0 1')   # rot(z,90deg)    ##TODO: get from DREAM3D pipeline  body_rot=retrieve_pose_from_dream3d(_)
            body_transl = np.matrix('1.96846; -2.55415; 0.6200')
            body_frame = tf.generateTransMatrix(body_rot, body_transl)
            print('Maze Origin Frame Calculated to be @ ', body_frame)

            # Rotate Path (as Rigid Body) according to body_frame
            path_via_fixed_frame = tf.convertPath2FixedFrame(path_as_tf_matrices, body_frame)

            # Convert Path of Transforms to Robot Poses
            new_path_poses = tf.convertPath2RobotPose(path_via_fixed_frame)
            starting_orientation = maze_stool_start[-4:]

            raw_input('>> Begin Solving maze <enter>')
            for node in new_path_poses:
                try:
                    # Overwrite Requested Orientation with that of Maze Starting Orientation
                    node[-4:] = starting_orientation

                    print('Moved to Pose:')
                    print(node)

                    robot_pointer.goto_Quant_Orient(node)

                    #raw_input('>> Next Pose <enter>')

                except KeyboardInterrupt:
                    return




        ############################
        # Close Vision Pipeline
        #camera.stopRSpipeline()

        # Move to Known Start Position: All-Zeros
        if motion_testing:
            raw_input('Go to Maze Overlook Crouch Position <enter>')
            robot_camera.goto_named_target("maze_overlook_crouch")


    except rospy.ROSInterruptException:
        # Close Vision Pipeline
        #camera.stopRSpipeline()
        return

    except KeyboardInterrupt:
        # Close Vision Pipeline
        #camera.stopRSpipeline()
        return


#####################################################
if __name__ == '__main__':
    main()
