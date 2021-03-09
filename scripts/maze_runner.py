#!/usr/bin/env python

#####################################################
##   Maze Runner: Move Robotic Arm Through Maze    ##
##                                                 ##
##   Capabilities Demonstrated                     ##
##   * Planar Maze Following                       ##
##                                                 ##
#####################################################

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, The Ohio State University
# Center for Design and Manufacturing Excellence (CDME)
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
# All rights reserved.
#
# Author: Adam Buynak

#####################################################


import numpy as np

from process_path import prepare_path_transforms_list
from process_path import prepare_path_tf_ready

from transformations import transformations

import robot_support


def retrieve_pose_from_dream3d():
  """
  Input Dream3D. Don't know what this looks like. 
  Place holder! Yay!
  """



def main():

  print("")
  print("----------------------------------------------------------")
  print("           Maze Runner       (TOP LEVEL)                  ")
  print("----------------------------------------------------------")
  print("Example developed by ACBUYNAK. Spring 2021")
  #print("Note: You will be unable to interrupt program if running\n from terminal. Alternatively use IDLE console.")
  #print("Press Enter to advance script when prompted.")
  print("")



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


############################3

  try:
    robot = moveManipulator()
    robot.set_accel(0.2)
    robot.set_vel(0.2)

    # Move to Known Start Position: All-Zeros
    raw_input('Go to All-Zeros Position <enter>')
    robot.goto_all_zeros()

    for msg in new_path_poses:
      print(msg)

      robot.goto_Quant_Orient(msg)
      time.sleep(0.5)

  except rospy.ROSInterruptException:
      return

  except KeyboardInterrupt:
      return

if __name__ == '__main__':
  main()