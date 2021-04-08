#! /usr/bin/python3

#####################################################
##         Dream3D Post-Processing Tool            ##
#####################################################
# Software License Agreement (Apache 2.0 License)   #
# Author: acbuynak                                  #
#####################################################


#### IMPORTS ####
import csv
import cv2
import os
import numpy as np


#### TOOLS ####
class Dream3DPostProcess(object):
    """
    Post-Processor for Dream3D Images
    """

    def __init__(self, userColorList, userInputList, maze_size_meters, tolerance=1000, mask=False, maskImg_MazeOnly=False, mzrun_ws_path=False):

        # Minimum Required
        self.dots = {}
        self.loadDots(userColorList, userInputList, tolerance)
        self.scale = self.calcScale(self.dots, maze_size_meters)
        self.rotMatrix = self.calcRotation(self.dots)
        self.mazeCentroid = self.calcMazeCentroid(self.dots, self.rotMatrix)


        # Additional Information
        if mask:
            path_image_cropped = self.calcMazeMask(self.dots, maskImg_MazeOnly, self.rotMatrix)
            filepath_maurer_path =  self.findMaurerPath(path_image_cropped, mzrun_ws_path)
            self.path_solution_list, self.path_solution_image = self.callPathSolver(filepath_maurer_path, mzrun_ws_path)

            print(self.path_solution_image)


    def loadDots(self, userColorList, userFileList, tolerance=50):
        """
        Load Dot's from Dictionary by processing FeatureData Generated by Dream3D Filter Pipeline
        :param userColorList:   Input List of Dot (color) names (order correlated)
        :param userFileList:    Input List of absolute file paths (order correlated)
        :param tolerance:       Minimum number of volume elements in a color 'blob' to be considered a dot
        :return self.dots:      3-element Dictionary (3dots) with list of centroid information in each
        """

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

                print('Found largest <' + str(colorName) + '> pixel group with <' + str(largest_volume) + '> elements.')

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

    def calcMazeCentroid(self, dots, rotMatrix):
        """
        Find Maze Centroid in Image using Centroids of the 3 Dots
        :param dots: Input Dictionary with three dot keys.
        :return rot_matrix: 3x3 Rotation Matrix.
        """
        
        # Convention:   Right-Hand-Rule
        #   Origin  :   @ GREEN
        #   X-Vector:   GREEN -> RED
        #   Y-VECTOR:   GREEN -> BLUE
    
        # Use Numpy for Calculation Convenience
    
        # Find Vectors
        side_gr = np.subtract(dots['red']['centroid'],    dots['green']['centroid'])
        side_gb = np.subtract(dots['blue']['centroid'],   dots['green']['centroid'])

        len_gr = np.linalg.norm(side_gr)
        len_gb = np.linalg.norm(side_gb)

        # todo: limited to 2D. Must use different method for 3D
        angle = np.arccos(rotMatrix[0,0])
        center_x = 0.5 * (len_gr*np.cos(angle) + len_gb*np.sin(angle))
        center_y = 0.5 * (len_gb*np.cos(angle) + len_gr*np.sin(angle))

        mazeCentroid = np.array([center_x, center_y])    
        
        return mazeCentroid

    def calcScale(self, dots, maze_size_meters):
        """
        Find scale (pixels vs world) by comparing pixel length of side of maze with real world length 
        :param dots: Input Dictionary with three dot keys.
        :param maze_size_meters: Tuple of length 2. <distance from green-red dot, distance from green-blue dots>
        :return: scale
        """
        side_gr = np.subtract(dots['red']['centroid'],    dots['green']['centroid'])
        side_gb = np.subtract(dots['blue']['centroid'],   dots['green']['centroid'])

        side_gr_length = np.linalg.norm(side_gr)
        side_gb_length = np.linalg.norm(side_gb)

        scale_gr = maze_size_meters[0] / side_gr_length
        scale_gb = maze_size_meters[1] / side_gb_length

        scale = np.average([scale_gr,scale_gb])

        print('\n DEBUG: Found maze real/px scale = ')
        print(scale)
        print('\n')

        return scale

    def calcMazeMask(self, dots, maskImg_MazeOnly, rotMatrix):
        """
        Generate a mask of region confined by a quadrilateral constrained by the three dot centroids
        :param dots: Input Dictionary with three dot keys.
        :param maskImg_MazeOnly: Absolute Path to 8-bit image (0-255) of the maze path
        :return: size in list [x,y,z]
        """
    
        axis_x = np.subtract(dots['red']['centroid'],    dots['green']['centroid'], where="array_like")
        axis_y = np.subtract(dots['blue']['centroid'],   dots['green']['centroid'])
        
        # Find Mask Corners
        corner_NW = dots['green']['centroid']
        corner_NE = dots['red']['centroid']
        corner_SW = dots['blue']['centroid']
        corner_SE = dots['blue']['centroid'] + axis_x   #potential issue spot for 3D in future
        #print(" > Four Corners Identified @ :", corner_NW, corner_NE, corner_SW, corner_SE)


        # Load Image as 8-bit, single channel
        img = cv2.imread(maskImg_MazeOnly, 0)

        # Build Mask
        height = img.shape[0]
        width = img.shape[1]

        mask = np.zeros((height, width), dtype=np.uint8)
        centroids = np.stack((corner_NW[:2], corner_NE[:2], corner_SE[:2], corner_SW[:2]), axis=0)
        cv2.fillPoly(mask, [centroids], (255))

        img_mask = cv2.bitwise_and(img,img,mask = mask)

        # Export Masked Image
        path, ext = os.path.splitext(maskImg_MazeOnly)
        path_new = "{path}_{uid}{ext}".format(path=path, uid="mask", ext=ext)
        #img_mask = cv2.cvtColor( img_mask, cv2.COLOR_RGB2GRAY)
        cv2.imwrite(path_new, img_mask)



        # Rotate Image
        # Use Transpose to get rotation opposite direction
        planar_rotMatrix = rotMatrix[:-1,:-1].T
        planar_transform = np.column_stack((planar_rotMatrix, np.array([width, height/2]).T))

        image_center = tuple(np.array(img_mask.shape[1::-1])/2)
        img_rotated = cv2.warpAffine(img_mask, planar_transform, img_mask.shape[1::-1], flags=cv2.INTER_NEAREST)

        # Export
        path_new = "{path}_{uid}{ext}".format(path=path, uid="mask_rot", ext=ext)
        cv2.imwrite(path_new, img_rotated)



        # Crop Image
        rect = cv2.boundingRect(np.argwhere(img_rotated == 255))
        x,y,w,h = rect
        img_crop = img_rotated[x:x+w, y:y+h].copy() #1 px buffer to ensure

        path_crop_img = "{path}_{uid}{ext}".format(path=path, uid="mask_rot_crop", ext=ext)
        #cv2.imwrite(path_crop_img, img_crop)   #todo: known to cause error 'floating point exception'



        # Debug by showing images
        # cv2.imshow("Imported Image: Maze Path w/ Envr Clutter", img)
        # cv2.imshow("Masked Maze Path", img_mask)
        # cv2.imshow("Masked Image Rotated", img_rotated)
        # cv2.imshow("Masked Image Cropped", img_crop)
        # cv2.waitKey(0)


        return path_crop_img
        


    def findMaurerPath(self, path_image_cropped, mzrun_ws_path):
        """
        Call Dream3D Maurer Filter Pipeline. Post-process generate image.
        :param path_image_cropped: Absolute file path to the cropped maze
        :param mzrun_ws_path: Absolute path to maze-runner package working directory
        :return: Saves new Image (maze_maurer_path.tiff)
        """

        import subprocess
        import json

        pipeline = os.path.dirname(mzrun_ws_path) +'/dream3d_pipelines/filter_maurer_path.json'
        filepath_maurer_path = str(mzrun_ws_path) + '/maze_maurer_path.tiff' 

        # Setup Dream3D / Simple Pipeline
        with open(pipeline, 'r') as jsonFile:
            data = json.load(jsonFile)

            data["0"]["FileName"] = path_image_cropped
            data["7"]["FileName"] = filepath_maurer_path

        with open(pipeline, 'w') as jsonFile:
            json.dump(data, jsonFile, indent=4)

        # Run Dream3D Pipeline
        d3d_output = open(mzrun_ws_path + '/temp_mauer_pipelineResult.txt', 'a')  # Workaround to supress output.
        subprocess.call(["/opt/dream3d/bin/PipelineRunner", "-p", pipeline], stdout=d3d_output, stderr=d3d_output)


        # Post Process Dream3D Generated Maurer Image
        img = cv2.imread(filepath_maurer_path, 0)

        # Find number of all-black rows on header
        first_row = 0
        whites = np.argwhere(img[first_row,:] == 255)
        while len(whites) == 0:
            first_row += 1
            whites = np.argwhere(img[first_row,:] == 255)

        # Force Single White pixel along top of maze
        first = True
        for i in whites:
            if first: first_col = i
            if not first:  img[0,i] = 0
            first = False

        # Draw WhiteLine straight up from the start pixel
        while first_row > 0:
            first_row -= 1
            img[first_row, first_col] = 255



        # Find first bottom row containing white pixels
        last_row = img.shape[0] - 1
        whites = np.argwhere(img[last_row,:] == 255)
        while len(whites) == 0:
            last_row -= 1
            whites = np.argwhere(img[last_row,:] == 255)

        # Draw WhiteLine straight down from all white pixels along bottom-most white pixel containing row
        while last_row < img.shape[0]-1:
            last_row += 1
            for col in whites:
                img[last_row, col] = 255



        cv2.imwrite(filepath_maurer_path, img)
        # cv2.imshow("Fixed Mauer Path Entrance", img)
        # cv2.waitKey(0)
    

        return filepath_maurer_path


    def callPathSolver(self, filepath_maurer_path, mzrun_ws_path):
        """
        Call Dream3D Maurer Filter Pipeline. Post-process generate image.
        :param path_image_cropped: Absolute file path to the cropped maze
        :param mzrun_ws_path: Absolute path to maze-runner package working directory
        :return: Filename (less extension)  (maze_maurer_path.tiff)
        """

        #todo: using this as a wrapper. In future, covert solver to a python class and import

        import subprocess

        solver = os.path.dirname(mzrun_ws_path) + '/include/mazesolving/solve.py'
        output_filename = str(mzrun_ws_path) + '/path_solved'

        solver_output = open(mzrun_ws_path + '/temp_mauer_pipelineResult.txt', 'a')  # Workaround to supress output.
        subprocess.call(["python", solver, filepath_maurer_path, output_filename], stdout=solver_output, stderr=solver_output)

        return (str(output_filename) + '.csv', str(output_filename) + '.tiff')
        
    


#### MAIN CODE ####

def main():
    print('\n--- Demo Code: "d3d_post_process.py" ---\n')

    # Demonstration Code:
    color_names = ["red", "green", "blue"]
    mzrun_ws_path = '/home/aims-zaphod/AA_DEVL/ws_mazerunner_multibot/src/maze-runner/mzrun_ws'
    centroid_filepaths = ['/home/aims-zaphod/AA_DEVL/ws_mazerunner_multibot/src/maze-runner/mzrun_ws/feature_redDot.csv',
                          '/home/aims-zaphod/AA_DEVL/ws_mazerunner_multibot/src/maze-runner/mzrun_ws/feature_greenDot.csv',
                          '/home/aims-zaphod/AA_DEVL/ws_mazerunner_multibot/src/maze-runner/mzrun_ws/feature_blueDot.csv'
                          ]
    mask_MazeOnly = '/home/aims-zaphod/AA_DEVL/ws_mazerunner_multibot/src/maze-runner/mzrun_ws/mask_MazeOnly.tiff'
    


    # Realworld Measurement (in meters)
    maze_size = [0.18, 0.18]

    d3d = Dream3DPostProcess(color_names, centroid_filepaths, maze_size, tolerance=10, mask=True, maskImg_MazeOnly=mask_MazeOnly, mzrun_ws_path=mzrun_ws_path)

    #print(d3d.dots)
    #print(d3d.rotMatrix)
    #print(d3d.mazeCentroid)
    print('\n--- Demo Code: Complete ---\n')

if __name__ == "__main__":
    main()