#!/usr/bin/env python3
#
# Support file for node: vision_processing
# 

#### IMPORTS ####
import csv
import cv2
import os
import numpy as np


#### TOOLS ####
class CONTEXTUALIZE_D3D_FEATURES():
    """
    Post-Processor for Dream3D Images
    """

    def __init__(self, userColorList, userInputList, maskImg_MazeOnly, maze_size_meters, tolerance=100):

        # Load Feature Data
        self.dots = {}
        self.loadDots(userColorList, userInputList, tolerance)

        # Load Feature Names
        self.red    = userColorList[0]
        self.green  = userColorList[1]
        self.blue   = userColorList[2]

        # Process
        self.scale 		  = self.calcScale(self.dots, maze_size_meters)
        self.rotMatrix 	  = self.calcRotation(self.dots)

        if self.rotMatrix is not None:
            self.mazeCentroid = self.calcMazeCentroid(self.dots, self.rotMatrix)
            self.path_image_cropped = self.calcMazeMask(self.dots, maskImg_MazeOnly, self.rotMatrix)
            return
        else:
            return None

    def loadDots(self, userColorList, userFileList, tolerance=50):
        """
        Load Dot's from Dictionary by processing FeatureData Generated by Dream3D Filter Pipeline
        :param userColorList:   Input List of Dot (color) names (order correlated)
        :param userFileList:    Input List of absolute file paths (order correlated)
        :param tolerance:       Minimum number of volume elements in a color 'blob' to be considered a dot
        :return self.dots:      3-element Dictionary (3dots) with list of centroid information in each
        """

        import csv
        import rclpy
        import numpy as np

        # Read in Relevant Feature Values from Dream3D Produced FeatureData.csv
        for colorName, file in zip(userColorList, userFileList):
            with open(file) as csvfile:
                read = csv.DictReader(csvfile, delimiter=',')

                # todo: not sure how to avoid 'looping' through a single row. Issue with unable to call elements. ex read[0]["Centroids_0"] is not possible.
                # todo: find better way to check that the correct blob is grabbed. Or how to handle when other objects of similar color on screen.

                largest_volume = tolerance
                largest_volume_index = 0
                centroid = np.array([0, 0, 0])

                for index, row_value in enumerate(read):
                    if int(row_value["Volumes"]) > largest_volume:
                        largest_volume = int(row_value["Volumes"])
                        largest_volume_index = index
                        
                        centroid[0] = float(row_value["Centroids_0"])
                        centroid[1] = float(row_value["Centroids_1"])
                        centroid[2] = float(row_value["Centroids_2"])  # not currently using 'Z-axis' but including for completeness.

                # print('Found largest <' + str(colorName) + '> pixel group with <' + str(largest_volume) + '> elements.')

            # Add Centroid Information to Dictionary
            self.dots[colorName] = {}
            self.dots[colorName]["centroid"] = centroid
        return self.dots

    def calcScale(self, dots, maze_size_meters):
        """
        Find scale (pixels vs world) by comparing pixel length of side of maze with real world length 
        :param dots: Input Dictionary with three dot keys.
        :param maze_size_meters: Tuple of length 2. <distance from green-red dot, distance from green-blue dots>
        :return: scale
        """
        # import rospy
        import numpy as np

        side_gr = np.subtract(dots[self.red]['centroid'],    dots[self.green]['centroid'])
        side_gb = np.subtract(dots[self.blue]['centroid'],   dots[self.green]['centroid'])

        side_gr_length = np.linalg.norm(side_gr)
        side_gb_length = np.linalg.norm(side_gb)

        scale_gr = maze_size_meters[0] / side_gr_length
        scale_gb = maze_size_meters[1] / side_gb_length

        scale = np.average([scale_gr,scale_gb])
        # print('DEBUG: Found maze real/px scale = ' + str(scale)) #DEBUG

        return scale

    def calcRotation(self, dots):
        """
        Find Image Rotation using Centroids of Each of 3 Dots
        :param dots: Input Dictionary with three dot keys.
        :return rot_matrix: 3x3 Rotation Matrix.
        """
        # import rospy

        # Convention:   Right-Hand-Rule
        #   Origin  :   @ GREEN
        #   X-Vector:   GREEN -> RED
        #   Y-VECTOR:   GREEN -> BLUE
    
        # Use Numpy for Calculation Convenience
        
        # Find Maze Axis Vectors
        axis_x = np.subtract(dots[self.red]['centroid'],    dots[self.green]['centroid'], where="array_like")
        axis_y = np.subtract(dots[self.blue]['centroid'],   dots[self.green]['centroid'])

        axis_x = axis_x / np.linalg.norm(axis_x,2)
        axis_y = axis_y / np.linalg.norm(axis_y,2)

        # Check if Vectors are sufficiently square to each other
        dot_prod = np.dot(axis_x, axis_y)
        if dot_prod < np.cos(np.deg2rad(45)):
            axis_z = np.cross(axis_x,axis_y)
            axis_z = axis_z / np.linalg.norm(axis_z,2)
        else:
            print("X & Y axis insufficiently square. Failed to assemble rotation matrix.")
            return None
    
        # Package into Rotation Matrix
        # https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Formalism_alternatives
        rot_matrix = np.stack([axis_x, axis_y, axis_z], axis=1)
        
        return rot_matrix

    def calcMazeCentroid(self, dots, rotMatrix):
        """
        Find Maze Centroid in Image using Centroids of the 3 Dots
        :param dots: Input Dictionary with three dot keys.
        :return rot_matrix: 3x3 Rotation Matrix.
        """
        # import rospy
        import numpy as np

        # Convention:   Right-Hand-Rule
        #   Origin  :   @ GREEN
        #   X-Vector:   GREEN -> RED
        #   Y-VECTOR:   GREEN -> BLUE
    
        assert rotMatrix.shape == (3, 3), "Assert: Rotation Matrix must be size (3,3)."
        assert np.isnan(rotMatrix).any() == False, "Assert: Rotation Matrix must not contan NaN values."

        # Find Vectors
        side_gr = np.subtract(dots[self.red]['centroid'],    dots[self.green]['centroid'])
        side_gb = np.subtract(dots[self.blue]['centroid'],   dots[self.green]['centroid'])

        len_gr = np.linalg.norm(side_gr)
        len_gb = np.linalg.norm(side_gb)

        # todo: limited to 2D. Must use different method for 3D
        # todo: this technique has inherent mathematical flaws as by using arccos(). Should use arctan().
        # angle = np.arccos(rotMatrix[0,0])
        angle = np.arctan2(rotMatrix[0,1], rotMatrix[0,0])
        center_x = 0.5 * (len_gr*np.cos(angle) + len_gb*np.sin(angle))
        center_y = 0.5 * (len_gb*np.cos(angle) + len_gr*np.sin(angle))

        mazeCentroid = np.array([center_x, center_y])    
        
        return mazeCentroid

    def calcMazeMask(self, dots, maskImg_MazeOnly, rotMatrix):
        """
        Generate a mask of region confined by a quadrilateral constrained by the three dot centroids
        :param dots: Input Dictionary with three dot keys.
        :param maskImg_MazeOnly: Absolute Path to 8-bit image (0-255) of the maze path
        :return: size in list [x,y,z]
        """

        import cv2
        import numpy as np
        # import rospy

        # Check Input
        assert rotMatrix.shape == (3, 3), "Assert: Rotation Matrix must be size (3,3)."
        assert np.isnan(rotMatrix).any() == False, "Assert: Rotation Matrix must not contan NaN values."

        axis_x = np.subtract(dots[self.red]['centroid'],    dots[self.green]['centroid'], where="array_like")
        axis_y = np.subtract(dots[self.blue]['centroid'],   dots[self.green]['centroid'])
        
        # Find Mask Corners
        corner_NW = dots[self.green]['centroid']
        corner_NE = dots[self.red]['centroid']
        corner_SW = dots[self.blue]['centroid']
        corner_SE = dots[self.blue]['centroid'] + axis_x   #TODO: potential issue spot for 3D in future
        #print(" > Four Corners Identified @ :", corner_NW, corner_NE, corner_SW, corner_SE)

        # Load Image as 8-bit, single channel
        img = cv2.imread(maskImg_MazeOnly, 0)

        # Build Mask
        height    = img.shape[0]
        width     = img.shape[1]
        mask      = np.zeros((height, width), dtype=np.uint8)
        centroids = np.stack((corner_NW[:2], corner_NE[:2], corner_SE[:2], corner_SW[:2]), axis=0)
        cv2.fillPoly(mask, [centroids], (255))

        img_mask = cv2.bitwise_and(img, img, mask = mask)

        # Export Masked Image
        path, ext = os.path.splitext(maskImg_MazeOnly)
        path_new = "{path}_{uid}{ext}".format(path=path, uid="mask", ext=ext)
        cv2.imwrite(path_new, img_mask)


        # Find Center of Maze Points
        x = np.mean(centroids[:,0])
        y = np.mean(centroids[:,1])
        center = (int(x), int(y))


        # Rotate Image about Center of Maze
        counterRotate = np.array([[0,-1],[1,0]]) * np.linalg.inv(rotMatrix)[:-1,:-1]
        rotAngle2D = np.arctan2(counterRotate[0,0], counterRotate[1, 0])
        # print("Z-Axis Angle Rotation Correction: " + str(int(np.degrees(rotAngle2D))) + " degrees")

        counterTransform = cv2.getRotationMatrix2D(center, np.degrees(rotAngle2D), scale=1.0)
        img_rotated = cv2.warpAffine(img_mask, counterTransform, img_mask.shape[1::-1], flags=cv2.INTER_NEAREST)


        # Export
        path_new = "{path}_{uid}{ext}".format(path=path, uid="mask_rot", ext=ext)
        cv2.imwrite(path_new, img_rotated)


        # Crop Image
        rect = cv2.boundingRect(np.argwhere(img_rotated == 255))
        x,y,w,h = rect
        img_crop = img_rotated[x:x+w, y:y+h].copy() #1 px buffer to ensure

        path_crop_img = "{path}_{uid}{ext}".format(path=path, uid="mask_rot_crop", ext=ext)
        cv2.imwrite(path_crop_img, img_crop)   #todo: known to cause error 'floating point exception'


        return path_crop_img

    def exportResults(self):
        return self.path_image_cropped