#!/usr/bin/env python

#####################################################
##   Visualizations Support Class                  ##
##                                                 ##
##   Construct Visuals                             ##
##   * Show path in 3D space                       ##
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


def plot_path_vectors(d2p, limits=None):
  """"
  Plots Path of Vectors
  :param d2p: Array where Row = Point, Columns = XYZ coordinates
  :param limits: Optional. List of three tuples for XYZ limits
  """

  import matplotlib.pyplot as plt
  from mpl_toolkits.mplot3d import Axes3D

  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

  # Origin
  ax.scatter(0, 0, 0)  # origin

  # Path Plotted
  for p in d2p:
    ax.scatter(p[0], p[1], p[2], marker="x")
  ax.plot(d2p[:, 0], d2p[:, 1], d2p[:, 2], color='b')

  # Assign Labels
  ax.set_xlabel('X-Axis')
  ax.set_ylabel('Y-Axis')
  ax.set_zlabel('Z-Axis')

  # Plot Ranges
#  if not limits is None:
#    xlim = limits[0]
#    ylim = limits[1]
#    zlim = limits[2]

#    ax.set_xlim(xlim)
#    ax.set_ylim(ylim)
#    ax.set_zlim(zlim)

  # Display Plot
  plt.show()

  # Close Figure
  #plt.close()


def plot_path_transforms(d2p):
  import numpy as np
  import matplotlib.pyplot as plt
  from mpl_toolkits.mplot3d import Axes3D

  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

  # Origin
  ax.scatter(0, 0, 0)  # origin

  # Collect Array of Just Vectors
  vectors = np.zeros((len(d2p),3))
  for i, transform in enumerate(d2p):
    vectors[i,:] = transform[:-1,3]

  # Path Plotted
  for p in d2p:
    ax.scatter(p[0,3], p[1,3], p[2,3], marker="x")
  ax.plot(vectors[:, 0], vectors[:, 1], vectors[:, 2], color='b')

  # Assign Labels
  ax.set_xlabel('X-Axis')
  ax.set_ylabel('Y-Axis')
  ax.set_zlabel('Z-Axis')

  # Display Plot
  plt.show()

  # Close Figure
  #plt.close()
