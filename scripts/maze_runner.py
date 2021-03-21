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


def retrieve_pose_from_dream3d(workspace_path, image_path):
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

  data["23"]["FeatureDataFile"] = path_redDot
  data["24"]["FeatureDataFile"] = path_greenDot
  data["25"]["FeatureDataFile"] = path_blueDot

  data["26"]["OutputFilePath"] = workspace_path + '/feature_redDot_array.csv'
  data["28"]["OutputFilePath"] = workspace_path + '/filter_irs_image.dream3d'

  data["31"]["FileName"] = workspace_path + '/mask_MazeOnly.tiff'
  data["34"]["FileName"] = workspace_path + '/mask_mostlyBlue.tiff'

  with open(pipeline, 'w') as jsonFile:
    json.dump(data, jsonFile)

  subprocess.call(["/opt/dream3d/bin/PipelineRunner", "-p", pipeline])

  # Post Process Features to find Rotation
  color_names = ["red", "green", "blue"]
  centroid_filepaths = [path_redDot, path_greenDot, path_blueDot]

  d3d = Dream3DPostProcess(color_names, centroid_filepaths, tolerance=80)
  rotationMatrix = d3d.rotMatrix
  centroid = d3d.mazeCentroid

  return centroid, rotationMatrix


class DataSaver(object):
  """
  All data saving functions wrapped in one tool.
  """
  def __init__(self, mzrun_ws, robot, camera):
    #Setup
    self.workspace = mzrun_ws
    self.robot = robot
    self.camera = camera

    setup_dict = {}
    setup_dict['counter'] = []
    setup_dict['poses'] = []
    setup_dict['images'] = []
    setup_dict['maze_rotationMatrix'] = []
    setup_dict['maze_centroid'] = []
    setup_dict['scale'] = []

    self.all_data = setup_dict

  def capture(self, img_counter, find_maze=False):
    img_path = self.camera.capture_singleFrame_color(self.workspace + '/' + str(img_counter))
    pose = self.robot.lookup_pose()

    if find_maze: 
      # Run Vision Pipeline, find Location & Rotation of Maze
      centroid, rotationMatrix = retrieve_pose_from_dream3d(self.workspace, img_path)
      self.save_data(self.workspace, img_counter, img_path, pose, centroid, rotationMatrix)
    else:
      self.save_data(self.workspace, img_counter, img_path, pose, np.array([0]), np.array([0]))

  def save_data(self, mzrun_ws, img_counter, img_path, pose, maze_centroid, maze_rotationMatrix) :
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

    with open(mzrun_ws + '/camera_poses.json', 'w') as outfile:
      json.dump(add_data, outfile, indent=4)

    self.all_data = add_data



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



  if False:
    # Set Maze to be Solved
    #solved_maze_path = raw_input('Input PreSolved Maze Path: ')
    print("Input Solve Maze Path:  'tiny_path_soln.csv'")
    solved_maze_path = 'tiny_path_soln.csv'

    # XY Process Path into Transformation List
    path_as_xyz = prepare_path_transforms_list(solved_maze_path)
    path_as_tf_matrices = prepare_path_tf_ready(path_as_xyz)

    # Find & Apply Rigid Body Transform to Maze 
    tf = transformations()
    body_rot = np.matrix('0 -1 0; 1 0 0; 0 0 1')   # rot(z,90deg)    ##TODO: get from DREAM3D pipeline  body_rot=retrieve_pose_from_dream3d(_)
    body_transl = np.matrix('0; 0; 0')
    body_frame = tf.generateTransMatrix(body_rot, body_transl)
    print('Maze Origin Frame Calculated to be @ ', body_frame)

    # Rotate Path (as Rigid Body) according to body_frame
    path_via_fixed_frame = tf.convertPath2FixedFrame(path_as_tf_matrices, body_frame)

    # Convert Path of Transforms to Robot Poses
    new_path_poses = tf.convertPath2RobotPose(path_via_fixed_frame)


  ############################
  #Debug Settings
  motion_testing = True
  
  try:

    # Setup Robot
    robot = moveManipulator()
    robot.set_accel(0.15)
    robot.set_vel(0.15)

    # Setup Camera
    camera = REALSENSE_VISION() # Higher Resolution made difficult... (set_color=[1280,720,6], set_depth=[1280,720,6])
    
    # Setup Working Directory
    dir_mzrun = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    mzrun_ws = dir_mzrun + '/mzrun_ws'

    try:
      os.makedirs(mzrun_ws, True)
    except OSError:
      if not os.path.isdir(mzrun_ws):
        raise
    os.chmod(mzrun_ws, 0777)

    # Setup Image Record JSON
    database = DataSaver(mzrun_ws, robot, camera)
    img = 0


    ############################
    # Move to Known Start Position: All-Zeros
    if motion_testing:
      raw_input('Go to Maze Overlook Crouch Position <enter>')
      robot.goto_named_target("maze_overlook_crouch")

    # Move to Start Position
    start_pose = robot.lookup_pose()
    start_pose.position.z -= 0.7
    if motion_testing:
      robot.goto_Quant_Orient(start_pose)

    # Record First Camera Set
    timer_wait() #ensure camera has had time to buffer initial frames
    database.capture(img, find_maze=False)


    # Increment Robot down for testing
    for img in range(1,4):

      # Move Robot
      raw_input('incr <enter>')
      tmp_pose = robot.lookup_pose()
      tmp_pose.position.z -= 0.2

      if motion_testing:
        robot.goto_Quant_Orient(tmp_pose)

      database.capture(img, find_maze=True)   


    ############################
    # Close Vision Pipeline
    camera.stopRSpipeline()

    # Move to Known Start Position: All-Zeros
    if motion_testing:
      raw_input('Go to Maze Overlook Crouch Position <enter>')
      robot.goto_named_target("maze_overlook_crouch")


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
