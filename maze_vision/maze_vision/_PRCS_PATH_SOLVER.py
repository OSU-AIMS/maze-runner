#!/usr/bin/env python3
#
# Support file for node: vision_processing
# 

import os
import cv2
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray


###################
## Class Methods ##
###################

def clean_maurer_img(self, maurer_image: np.ndarray) -> str:
    """
    Post-process image generated by the Dream3D Maurer Filter Pipeline.

    :param maurer_image_filepath: OpenCV Image D3D Maurer filtered maze
    :return: Saves new Image (maze_maurer_path.tiff)
    """

    # Find number of all-black rows on header
    first_row = 0
    whites = np.argwhere(maurer_image[first_row,:] == 255)
    while len(whites) == 0:
        first_row += 1
        whites = np.argwhere(maurer_image[first_row,:] == 255)

    # Force Single White pixel along top of maze
    first = True
    for i in whites:
        if first: first_col = i
        if not first:  maurer_image[0,i] = 0
        first = False

    # Draw WhiteLine straight up from the start pixel
    while first_row > 0:
        first_row -= 1
        maurer_image[first_row, first_col] = 255

    # Find first bottom row containing white pixels
    last_row = maurer_image.shape[0] - 1
    whites = np.argwhere(maurer_image[last_row,:] == 255)
    while len(whites) == 0:
        last_row -= 1
        whites = np.argwhere(maurer_image[last_row,:] == 255)

    # Draw WhiteLine straight down from all white pixels along bottom-most white pixel containing row
    while last_row < maurer_image.shape[0]-1:
        last_row += 1
        for col in whites:
            maurer_image[last_row, col] = 255

    # Debug
    # filepath_maurer_path = os.path.join(self.dir_wksp, 'maze_maurer_cleaned.tiff')
    # cv2.imwrite(filepath_maurer_path, maurer_image)
    # cv2.imshow("Fixed Mauer Path Entrance", maurer_image)
    # cv2.waitKey(0)

    return maurer_image


def convert_path_list_to_img_px_coords(self, transform_: np.ndarray, path: list) -> list:
    """
    Converts a list of tuples (row,col) in maze coordinate frame into the img coordinate frame.
    All input and output coordinates in integer pixel dimensions.
    
    :return: List of tuples representing the path's nodes in order wrt the color image frame (top-left corner of image)
    """ 
    path_color_img_coords = []
    for node in path:
        new_node = np.matmul(transform_, (node[0], node[1], 0, 1)).astype('int') #pixel coordinates MUST be intergers
        path_color_img_coords.append(new_node)

    return path_color_img_coords


def transform_path_to_pose_array(self, path: list, intrinsics: CameraInfo, dmap: np.ndarray) -> PoseArray:
    """
    Converts a list of tuples (row,col) in pixel dimensions to a ROS msg PoseArray format.
    Transforms from image pixel reference frame to real-world units in the camera's color-optical reference frame. 
    
    :return: PoseArray msg in meters wrt to the camera's color optical reference frame.
    """

    # Setup ROS msg structure
    mpa = PoseArray()
    mpa.header = Header(frame_id = 'rs1_color_optical_frame')
    # mpa.header = Header(frame_id = 'rs1_color_frame')

    # Loop through all nodes
    for node in path:

        # Depth Map Lookup
        depth = dmap[node[1], node[0]]  # native unit is millimeters

        # Transform
        x,y,z = self.transform_pixel_to_point(node, intrinsics, depth)

        # Populate ROS msg structure
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        mpa.poses.append(p)

    return mpa


def transform_pixel_to_point(self, node, intrinsics, depth) -> tuple:
    """
    Transforms a single 2D pixel into a 3D point relative to camera frame using input intrinsics.
    """

    # Transform into real world coordinates
    ppx = intrinsics.k[2]
    ppy = intrinsics.k[5]
    fx = intrinsics.k[0]
    fy = intrinsics.k[4]

    x = (node[0] - ppx) / fx
    y = (node[1] - ppy) / fy

    # Distortion Correction
    # Distortion model for Intel Realsense D400 series is 'Plumb Bob' (aka. 'Brown-Conrady') model.
    # No additional calculation needed.

    # Transform & Convert to Meters
    x = depth * x  * 0.001   #convert to meters
    y = depth * y  * 0.001
    z = depth * 0.001

    return (x,y,z)
