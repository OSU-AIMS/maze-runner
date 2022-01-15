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
        json.dump(data, jsonFile, indent=4, sort_keys=True)

    # Workaround to supress output.
    d3d_output = open(workspace_path + '/temp_pipelineResult.txt', 'a')
    subprocess.call(["/opt/dream3d/bin/PipelineRunner", "-p", pipeline], stdout=d3d_output, stderr=d3d_output)

    featureData_dots_filepaths = [path_redDot, path_greenDot, path_blueDot]

    print(">> Dream3D Pipeline Runner Complete")
    return featureData_dots_filepaths




def characterize_maze(camera, workspace_path, img_id, maze_size, featureData_dots_filepaths, path_depth_npy):


    # Post Process Features to find Rotation, mazeCentroid, mazeScale
    color_names = ["red", "green", "blue"]
    mask_MazeOnly = str(workspace_path) + '/mask_MazeOnly.tiff' #todo: hardcoded
    d3d = Dream3DPostProcess(color_names, featureData_dots_filepaths, maze_size, tolerance=80, mask=True, maskImg_MazeOnly=mask_MazeOnly, mzrun_ws_path=workspace_path)

    rotationMatrix = d3d.rotMatrix
    mazeCentroid = d3d.mazeCentroid
    mazeScale = d3d.scale

    mazeSolutionList = d3d.path_solution_list

    dots = d3d.dots.copy()


    # Apply Depth Information
    depth_array = np.load(str(workspace_path) + '/' + str(img_id) + '_depth.npy')
    #depth_array = np.load(path_depth_npy)

    # For Visuals & Debugging
    import cv2
    cv2.imwrite(str(workspace_path)+"/depth_data.tiff", depth_array)


    for color in color_names:

        pixel_coord = [dots[color]['centroid'][0], dots[color]['centroid'][1]]

        # Raw Value Pulled from Depth.
        # todo: Not corrected for length, do this in future!
        # todo: calculate average depth value about local region
        depth = depth_array[dots[color]['centroid'][1], dots[color]['centroid'][0]].astype(float) / 1000

        # Transform Dots into World Coordinates (meters)
        x = (pixel_coord[0] - camera.intrin_color.ppx)/camera.intrin_color.fx *depth
        y = (pixel_coord[1] - camera.intrin_color.ppy)/camera.intrin_color.fy *depth
        z = depth

        dots[color]['centroid_camera_frame'] = [0,0,0] #temporary filler
        dots[color]['centroid_camera_frame'][0] = x
        dots[color]['centroid_camera_frame'][1] = y
        dots[color]['centroid_camera_frame'][2] = z

        # print("----------------------------")
        # print("REAL DOT COORDINATES", color)
        # print(x,y,z)
        # print('')

    ## Maze Body Frame (origin @ NW dot)(green dot)
    from transformations import transformations
    tf_tool = transformations()

    #todo: Z-Axis inhereted from Camera Ref Frame? 
    mazeOrigin = dots['green']['centroid_camera_frame']
    body_frame = tf_tool.generateTransMatrix(rotationMatrix, mazeOrigin)

    print('\nBody Frame')
    print(' --- if issues, debug here first. Not confident correctly setup. Check translation values')
    print(body_frame)

    # Create list of lists.. [[x,y,z], [..], [..]] 
    dots_list = []
    dots_list.append(dots['red']['centroid_camera_frame'])
    dots_list.append(dots['green']['centroid_camera_frame'])
    dots_list.append(dots['blue']['centroid_camera_frame'])
    print(dots_list)

    print(">> Characterize Maze, Complete.")
    return mazeCentroid, rotationMatrix, mazeScale, mazeOrigin, mazeSolutionList, body_frame, dots_list


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
        setup_dict['maze_soln_filepath'] = []
        setup_dict['tf_camera2world'] = []
        setup_dict['tf_body2camera'] = []
        setup_dict['dots'] = []

        self.all_data = setup_dict


    def capture(self, img_counter, find_maze=False):
        #path_img = self.camera.capture_singleFrame_color(self.workspace + '/' + str(img_counter))
        path_img, path_depth_npy = self.camera.capture_singleFrame_alignedRGBD(self.workspace + '/' + str(img_counter))
        pose = self.robot.lookup_pose()

        # Calculate Camera Transformation (relative to world frame)
        import subprocess
        tf_filename = "tf_camera2world.npy"
        tf_listener = os.path.dirname(self.workspace) +'/nodes/tf_origin_camera_subscriber.py'
        subprocess.call([tf_listener, self.workspace, tf_filename])

        tf_list = np.load(str(self.workspace) + '/' + tf_filename)
        tf_camera2world = quant_pose_to_tf_matrix(tf_list)


        if find_maze: 
            # Run Vision Pipeline, find Location & Rotation of Maze
            featureData_dots_filepaths = retrieve_pose_from_dream3d(self.workspace, path_img, 1000)
            centroid, rotationMatrix, scale, mazeOrigin, mazeSolutionList, tf_body2camera, dots  = characterize_maze(self.camera, self.workspace, img_id=img_counter, maze_size=self.maze_size, featureData_dots_filepaths=featureData_dots_filepaths, path_depth_npy=path_depth_npy)

            self.save_data(self.workspace, img_counter, path_img, pose, centroid, rotationMatrix, scale, mazeOrigin, mazeSolutionList, tf_camera2world, tf_body2camera, dots)
        else:
            self.save_data(self.workspace, img_counter, path_img, pose, np.array([0]), np.array([0]), 0, np.array([0]), 'N/A', tf_camera2world, np.array([0]), np.array([0]))


    def save_data(self, mzrun_ws, img_counter, path_img, pose, maze_centroid, maze_rotationMatrix, scale, mazeOrigin, mazeSolutionList, tf_camera2world, tf_body2camera, dots) :
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
        add_data['images'].append(path_img)
        add_data['poses'].append(json_message_converter.convert_ros_message_to_json(pose))
        add_data['maze_centroid'].append(np.ndarray.tolist(maze_centroid))
        add_data['maze_rotationMatrix'].append(np.ndarray.tolist(maze_rotationMatrix))
        add_data['scale'].append(scale)
        add_data['maze_origin'].append(mazeOrigin)
        add_data['maze_soln_filepath'].append(str(mazeSolutionList))
        add_data['tf_camera2world'].append(np.ndarray.tolist(tf_camera2world))
        add_data['tf_body2camera'].append(np.ndarray.tolist(tf_body2camera))
        add_data['dots'].append(dots)

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
            'maze_soln_filepath' : self.all_data['maze_soln_filepath'][-1],
            'tf_camera2world' : self.all_data['tf_camera2world'][-1], 
            'tf_body2camera' : self.all_data['tf_body2camera'][-1],
            'dots' : self.all_data['dots'][-1],
        }

        return latest_data


def quant_pose_to_tf_matrix(quant_pose):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """

    # Extract Translation
    r03 = quant_pose[0]
    r13 = quant_pose[1]
    r23 = quant_pose[2]


    # Extract the values from Q
    q0 = quant_pose[3]
    q1 = quant_pose[4]
    q2 = quant_pose[5]
    q3 = quant_pose[6]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    tf_matrix = np.array([ [r00, r01, r02, r03],
                           [r10, r11, r12, r13],
                           [r20, r21, r22, r23],
                           [  0,   0,   0,  1 ]])
                            
    return tf_matrix


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
    motion_testing      = True
    unknown_maze_locn   = True
    path_testing        = True

    fixed_path          = False #demo for a presolved maze path placed on the stool

    try:

        # Setup Robot Planning Groups
        robot_camera = moveManipulator(eef='r2_eef_camera')
        robot_camera.set_accel(0.15)
        robot_camera.set_vel(0.20)
        robot_pointer = moveManipulator(eef='r2_eef_pointer')
        robot_pointer.set_accel(0.05)
        robot_pointer.set_vel(0.05)

        # Transformations Tool Class
        tf_tool = transformations()


        # Setup Camera
        camera = REALSENSE_VISION(set_color=[640,480,30], set_depth=[640,480,30], max_distance=5.0) # Intentionally setting this specific resolution so pixel counts can be controlled easier
        
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
        database = DataSaver(mzrun_ws, robot_camera, camera)
        img = 0


        ############################
        # Move to Known Start Position: All-Zeros
        if motion_testing:
            raw_input('>> Go to Maze Overlook Crouch Position <enter>')
            robot_camera.goto_named_target("maze_overlook_crouch")
            print()

        # Testing: Increment Robot Down from overloop position
        if unknown_maze_locn:

            # Move to Start Position
            raw_input('>> Go to Low Crouch Position <enter>')
            start_pose = robot_camera.lookup_pose()
            start_pose.position.z -= 0.7
            robot_camera.goto_Quant_Orient(start_pose)

            # Record First Camera Set
            raw_input('>> Start Finding Maze Path? <enter>')
            timer_wait() #ensure camera has had time to buffer initial frames

            database.capture(img, find_maze=True)   
            latest_data = database.last_recorded()

            last_mazeOrigin       = latest_data['maze_origin']
            last_scale            = latest_data['scale']
            last_tf_camera2world  = np.array(latest_data['tf_camera2world'])
            last_tf_body2camera   = np.array(latest_data['tf_body2camera'])

            print('\n<< Last Maze Information')
            print('last_origin',last_mazeOrigin)
            print('last_scale',last_scale)

            print('last_tf_camera2world (fixed)')
            rot_camera_hardcode  = np.array([[0,-1,0],[-1,0,0],[0,0,-1]])
            translate            = last_tf_camera2world[:-1,-1].tolist()
            last_tf_camera2world = tf_tool.generateTransMatrix(rot_camera_hardcode, translate)
            print(np.around(last_tf_camera2world,2))

            print('last_tf_body2camera')
            print(last_tf_body2camera)

            print('\n<< Maze Body Frame in World Coordinates')
            tf_maze2world = np.matmul(last_tf_camera2world, last_tf_body2camera)
            print(np.around(tf_maze2world,2))




        if path_testing and unknown_maze_locn and False:
            print("\n<< Show Locator Dots")
            latest_data = database.last_recorded
            dots = latest_data['dots']

            # Transform
            dots_as_tf_matrices = prepare_path_tf_ready(dots_list)

            # Find & Apply Rigid Body Transform to Maze 
            # pull from corrected tf in last section
            print('<< Using Maze Origin Frame @ ', tf_maze2world)

            # Rotate Path (as Rigid Body) according to body_frame
            dots_via_fixed_frame = tf_tool.convertPath2FixedFrame(dots_as_tf_matrices, tf_maze2world)

            # Convert Path of Transforms to Robot Poses
            dots_as_path_poses = tf_tool.convertPath2RobotPose(dots_via_fixed_frame)
            orientation = [0.97, 0.242, 0.014, 0.012]

            raw_input('>> Begin Showing Maze Dots <enter>')
            for node in dots_as_path_poses:
                try:
                    # Overwrite Requested Orientation with that of Maze Starting Orientation
                    node[-4:] = orientation

                    # Liftoff Surface for testing purposes
                    node[2] += 0.1

                    print('Moved to Dot @')
                    print(node)

                    robot_pointer.goto_Quant_Orient(node)

                    raw_input('>> Next Dot <enter>')

                except KeyboardInterrupt:
                    break
                    return






        if path_testing and unknown_maze_locn and True:
            print("\n<< Begin Path Testing")

            #  Load Maze Path & Scale
            latest_data = database.last_recorded()
            solved_maze_path = latest_data['maze_soln_filepath']
            last_scale = latest_data['scale']

            # XY Process Path into Transformation List
            path_as_xyz = prepare_path_transforms_list(solved_maze_path, scaling_factor=last_scale)
            path_as_tf_matrices = prepare_path_tf_ready(path_as_xyz)

            # Find & Apply Rigid Body Transform to Maze 
            # pull from corrected tf in last section
            print('<< Using Maze Origin Frame @ ', tf_maze2world)

            # Rotate Path (as Rigid Body) according to body_frame
            path_via_fixed_frame = tf_tool.convertPath2FixedFrame(path_as_tf_matrices, tf_maze2world)

            # Convert Path of Transforms to Robot Poses
            new_path_poses = tf_tool.convertPath2RobotPose(path_via_fixed_frame)
            orientation = [0.97, 0.242, 0.014, 0.012]
            index =0

            raw_input('>> Begin Solving maze <enter>')
            for node in new_path_poses:
                try:
                    # Overwrite Requested Orientation with that of Maze Starting Orientation
                    node[-4:] = orientation
                    # Liftoff Surface for testing purposes
                    node[2] += 0.04

                    print('Moved to Pose:')
                    print(node)

                    robot_pointer.goto_Quant_Orient(node)

                    if index == 0:
                        raw_input("wait")
                        index +=1
                    #raw_input('>> Next Pose <enter>')

                except KeyboardInterrupt:
                    break
                    return







        # Path Following Demo:
        if fixed_path:

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
            body_rot = np.matrix('1 0 0; 0 1 0; 0 0 1')   # rot(z,90deg)    ##TODO: get from DREAM3D pipeline  body_rot=retrieve_pose_from_dream3d(_)
            body_transl = np.matrix('1.96846; -2.55415; 0.6200')
            body_frame = tf_tool.generateTransMatrix(body_rot, body_transl)
            print('Maze Origin Frame Calculated to be @ ', body_frame)

            # Rotate Path (as Rigid Body) according to body_frame
            path_via_fixed_frame = tf_tool.convertPath2FixedFrame(path_as_tf_matrices, body_frame)

            # Convert Path of Transforms to Robot Poses
            new_path_poses = tf_tool.convertPath2RobotPose(path_via_fixed_frame)
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
            # raw_input('Go to Maze Overlook Crouch Position <enter>')
            # robot_camera.goto_named_target("maze_overlook_crouch")
            print()


    except rospy.ROSInterruptException:
        # Close Vision Pipeline
        camera.stopRSpipeline()
        return

    except KeyboardInterrupt:
        # Close Vision Pipeline
        camera.stopRSpipeline()
        return


#####################################################
if __name__ == '__main__':
    main()
