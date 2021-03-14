#! /usr/bin/python3

#####################################################
##         Dream3D Post-Processing Tool            ##
#####################################################
# Software License Agreement (Apache 2.0 License)   #
# Author: acbuynak                                  #
#####################################################


#### IMPORTS ####
import csv
import numpy as np


#### TOOLS ####
class Dream3DPostProcess(object):
    """
    Post-Processor for Dream3D Images
    """

    def __init__(self, userColorList, userInputList):

        # Minimum Required
        self.dots = {}
        self.loadDotsFromDict(userColorList, userInputList)
        
        # Additional Information
        self.rotMatrix = self.calcRotation(self.dots)
        self.calcMazeMask(self.dots)


    def loadDotsFromDict(self, userColorList, userFileList, tolerance=1000):
        """
        Load Dot's from Dictionary by processing FeatureData Generated by Dream3D Filter Pipeline
        :param userColorList:   Input List of Dot (color) names (order correlated)
        :param userFileList:    Input List of absolute file paths (order correlated)
        :param tolerance:       Minimum number of volume elements in a color 'blob' to be considered a dot
        :return self.dots:      Adds centroid information
        """

        # Read in Relevant Feature Values from Dream3D Produced FeatureData.csv
        for colorName, file in zip(userColorList, userFileList):
            with open(file) as csvfile:
                read = csv.DictReader(csvfile, delimiter=',')

                # todo: not sure how to avoid 'looping' through a single row. Issue with unable to call elements. ex read[0]["Centroids_0"] is not possible.
                # todo: find better way to check that the correct blob is grabbed. Or how to handle when other objects of similar color on screen.
                for row in read:
                    if int(row["Volumes"]) > tolerance:
                        print('Found <' + str(colorName) + '> group greater than <' + str(tolerance) + '> volumes. If this line is printed more than once per color, error.')
                        centroid = np.array([0, 0, 0])
                        centroid[0] = float(row["Centroids_0"])
                        centroid[1] = float(row["Centroids_1"])
                        centroid[2] = float(row["Centroids_2"])  # not currently using 'Z-axis' but including for completeness.
    
            # Add Centroid Information to Dictionary
            self.dots[colorName] = {}
            self.dots[colorName]["centroid"] = centroid

    def calcRotation(self, dots):
        """
        Find Image Rotation using Centroids of Each of 3 Dots
        :param dots: Input Dictionary with three dot keys.
        :return rot_matrix: 3x3 Rotation Matrix.
        """
        
        # Convention:   Right-Hand-Rule
        #   Origin  :   @ GREEN
        #   X-Vector:   GREEN -> RED
        #   Y-VECTOR:   GREEN -> BLUE
    
        # Use Numpy for Calculation Convenience
    
        # Find Vectors
        axis_x = np.subtract(dots['red']['centroid'],    dots['green']['centroid'], where="array_like")
        axis_y = np.subtract(dots['blue']['centroid'],   dots['green']['centroid'])
        axis_z = np.cross(axis_x,axis_y)
    
        # Normalize Everything
        axis_x = axis_x / np.linalg.norm(axis_x,2)
        axis_y = axis_y / np.linalg.norm(axis_y,2)
        axis_z = axis_z / np.linalg.norm(axis_z,2)
    
        # Package into Rotation Matrix
        # https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Formalism_alternatives
        rot_matrix = np.stack([axis_x, axis_y, axis_z],axis=1)
        
        return rot_matrix

    def calcMazeMask(self, dots):
        """
        Generate a mask of region confined by a quadrilateral constrained by the three dot centroids
        :param dots: Input Dictionary with three dot keys.
        :return: size in list [x,y,z]
        """
    
        axis_x = np.subtract(dots['red']['centroid'],    dots['green']['centroid'], where="array_like")
        axis_y = np.subtract(dots['blue']['centroid'],   dots['green']['centroid'])
        
        # Find Mask Corners
        corner_green1 = dots['green']['centroid']
        corner_red    = dots['red']['centroid']
        corner_blue   = dots['blue']['centroid']
        corner_green2 = dots['blue']['centroid'] + axis_x
        print(" > Four Corners Identified @ :", corner_green1, corner_red, corner_blue, corner_green2)
        
        # Find Max Reach
        max_x = max(corner_green1[0], corner_red[0], corner_blue[0], corner_green2[0])
        max_y = max(corner_green1[1], corner_red[1], corner_blue[1], corner_green2[1])
        max_z = max(corner_green1[2], corner_red[2], corner_blue[2], corner_green2[2])
        print(" > Max Axial Reach @ x,y,z : ", max_x, max_y, max_z)
        
        # Setup Mask Array
        mask = np.zeros((max_x, max_y))
        print('Mask Shape', mask.shape)
        
        print(axis_x)
        print(axis_y)
        
        # Edge Lines (left & right)
        if not axis_x[1]:
            #todo: fix such that the last value is included in range.
            edge_left_x = np.arange(corner_green1[0], corner_blue[0], 1)
            edge_left_y = np.arange(corner_green1[1], corner_blue[1], 1)
            edge_right_x = np.arange(corner_red[0], corner_green2[0], 1)
            edge_right_y = np.arange(corner_red[1], corner_green2[1], 1)
            
            print("Edge lines")
            print(edge_left_x)
            print(edge_left_y)
            print(edge_right_x)
            print(edge_right_y)
        
        # Build Mask
        #for row in
        #todo: finish writing mask code
        
        return "INCOMPLETE"
        
        
    


#### MAIN CODE ####

def main():
    # Demonstration Code:
    color_names = ["red", "green", "blue"]
    centroid_filepaths = ['/tmp/SAMPLE/FeatureData_Red.csv',
                          '/tmp/SAMPLE/FeatureData_Green.csv',
                          '/tmp/SAMPLE/FeatureData_Blue.csv'
                          ]

    d3d = Dream3DPostProcess(color_names, centroid_filepaths)

    print('')
    #print(d3d.dots)
    #print(d3d.rotMatrix)


if __name__ == "__main__":
    main()