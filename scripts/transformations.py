#!/usr/bin/env python

#####################################################
##   Transformations Support Class                 ##
##                                                 ##
##   Homogeneous Transformation Matrices           ##
##   * Body Path to Fixed Frame conversion tool    ##
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


class transformations:
  """ Class Container for Rigid Body Transformations """

  def generateTransMatrix(self, matr_rotate, matr_translate):
    """
    Convenience Function which accepts two inputs to output a Homogeneous Transformation Matrix
    Intended to function for 3-dimensions frames ONLY
    """

    ## If Translation Matrix is List, Convert
    if type(matr_translate) is list:
      matr_translate = np.matrix(matr_translate)
      print("Changed translation vector from input 'list' to 'np.matrix'")

    ## Evaluate Inputs. Check if acceptable size.
    if not matr_rotate.shape == (3, 3):
      raise Exception("Error Generating Transformation Matrix. Incorrectly sized inputs.")
    if not matr_translate.size == 3:
      raise Exception("Error Generating Transformation Matrix. Translation Vector wrong size.")

    ## Reformat Inputs to common shape
    if matr_translate.shape == (1, 3):
      matr_translate = np.transpose(matr_translate)
      print("Transposed input translation vector")

    ## Build Homogeneous Transformation matrix using reformatted inputs
    # Currently includes flexibility to different sized inputs. Wasted process time, but flexible for future.
    # Assigns bottom right corner as value '1'
    new_transformMatrix = np.zeros((4,4))
    new_transformMatrix[0:0+matr_rotate.shape[0], 0:0+matr_rotate.shape[1]] = matr_rotate
    new_transformMatrix[0:0+matr_translate.shape[0], 3:3+matr_translate.shape[1]] = matr_translate
    new_transformMatrix[new_transformMatrix.shape[0]-1,new_transformMatrix.shape[1]-1] = 1

    ## Return result
    return new_transformMatrix

  def translate(self, axis, dist):
    """ Translates points along a specified axis by the specified distance """
    if axis not in ['x', 'X', 'y', 'Y', 'z', 'Z', 'n', 'N']:
      print("[WARN] Invalid axis in Plane.translate(), original list was not changed")
      return

    translate = np.zeros((3,1))
    for p in translate:
      if axis in ['x', 'X']:
        p[0] += dist
      elif axis in ['y', 'Y']:
        p[1] += dist
      elif axis in ['z', 'Z']:
        p[2] += dist
    return translate

  def rotate(self, axis, angle, rad=False):
    """ Generates a Rotation matrix about specified axis by the specified angle """

    if axis not in ['x', 'X', 'y', 'Y', 'z', 'Z']:
      print("[WARN] Invalid axis in Plane.rotate()")
      return

    if not rad:
      angle *= np.pi / 180

    rot = np.identity(3)
    if axis in ['x', 'X']:
      rot[1][1] = np.cos(angle)
      rot[1][2] = np.sin(angle) * -1
      rot[2][1] = np.sin(angle)
      rot[2][2] = np.cos(angle)
    elif axis in ['y', 'Y']:
      rot[0][0] = np.cos(angle)
      rot[0][2] = np.sin(angle)
      rot[2][0] = np.sin(angle) * -1
      rot[2][2] = np.cos(angle)
    elif axis in ['z', 'Z']:
      rot[0][0] = np.cos(angle)
      rot[0][1] = np.sin(angle) * -1
      rot[1][0] = np.sin(angle)
      rot[1][1] = np.cos(angle)

    return rot

  def convertPath2FixedFrame(self, path_body, frame_body, frame_fixed=np.identity((4))):
    """
    Function for mapping a Path (type: List) onto a Fixed Frame (np. Homogenous Matrix)
    :param path_body: Python List of Transformation Matrices for each point along a path
    :param frame_body: Transformation matrix for the body
    :param frame_fixed: Defaults to Identity Matrix (or robot's fixed frame)
    """

    ## Find Arb Frame Defined By Fixed Frame
    frame_b2f = np.matmul(frame_body, frame_fixed)
    # print('frame_b2f')
    # print(frame_b2f)

    ## Convert Path Frames to be defined by Fixed Frame
    path_fixed = []
    for point in path_body:
      path_fixed.append(np.matmul(point, frame_b2f))

    return path_fixed

##########################

def main():
  ## Imports
  import numpy as np

  ## Instantiate Class
  tf = transformations()


  ## Fixed Frame (robot base_link frame)
  frame_fixed = np.identity((4))


  ## Arbitrary Body Frame Variables
  arb_matrix = np.matrix('0 -1 0; 1 0 0; 0 0 1')
  arb_vector = np.matrix('0; 10; 0')

  frame_arb = tf.generateTransMatrix(arb_matrix, arb_vector)
  #print(frame_arb)


  ## Arbitrary Path (defined as Transformation Matrices)
  path_arb = [
    np.matrix('1 0 0 0; 0 1 0 2; 0 0 1 0; 0 0 0 1'),
    np.matrix('1 0 0 1; 0 1 0 2; 0 0 1 0; 0 0 0 1'),
    np.matrix('1 0 0 2; 0 1 0 2; 0 0 1 0; 0 0 0 1'),
    np.matrix('1 0 0 2; 0 1 0 3; 0 0 1 0; 0 0 0 1')
  ]


  ## Map Path to Fixed Frame
  new_fixed_path = tf.convertPath2FixedFrame(path_arb, frame_arb)
  print(new_fixed_path)


if __name__ == '__main__':
  main()