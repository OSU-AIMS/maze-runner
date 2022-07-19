#!/usr/bin/env python3
#
# Support file for node: vision_processing
# 

import os
import cv2
import numpy as np
import subprocess
import json


###################
## Class Methods ##
###################

def runD3D_maze_locators(self, fpath_image, dot_names, locator_scalar_tolerance=1000, local_debug="False") -> tuple:
    """
    Processor caller for preconfigured Dream3D pipeline.
    Requires: os, subprocess, json, rospy, rospkg

    :param fpath_image: Absolute filepath to maze image
    :param dot_names:   List of Strings, 3-element. Order MUST be red, green, blue. But exact names may be changed.
    return: Centroid and Size of red, green, blue color clusters
    """

    pipeline    = os.path.join(self.dir_resources, 'pipeline_rgb_fiducial.json')

    fpath_redDot     = os.path.join(self.dir_wksp, 'feature_' + dot_names[0] + '.csv')
    fpath_greenDot   = os.path.join(self.dir_wksp, 'feature_' + dot_names[1] + '.csv')
    fpath_blueDot    = os.path.join(self.dir_wksp, 'feature_' + dot_names[2] + '.csv')
    fpath_maze       = os.path.join(self.dir_wksp, 'mask_MazeOnly.tiff')


    # Setup & Run Dream3D / Simple Pipeline
    with open(pipeline, 'r') as jsonFile:
        data = json.load(jsonFile)
        data["00"]["FileName"] = fpath_image

        # Feature Segmenter Settings
        data["18"]["ScalarTolerance"] = locator_scalar_tolerance
        data["19"]["ScalarTolerance"] = locator_scalar_tolerance
        data["20"]["ScalarTolerance"] = locator_scalar_tolerance

        # Desired Outputs
        data["27"]["FeatureDataFile"] = fpath_redDot    # RED
        data["28"]["FeatureDataFile"] = fpath_greenDot  # GREEN
        data["29"]["FeatureDataFile"] = fpath_blueDot   # BLUE
        data["33"]["FileName"]        = fpath_maze      # MAZE

        # Debugging Tools
        data["34"]["Filter_Enabled"] = local_debug
        data["34"]["OutputFile"]     = os.path.join(self.dir_log, 'pipeline_filter_irs_image.dream3d')

    # Write out updated Json
    with open(pipeline, 'w') as jsonFile:
        json.dump(data, jsonFile, indent=4, sort_keys=True)

    # Workaround to supress output.
    d3d_output_path = os.path.join(self.dir_log, 'log_D3D_mazeLocators.txt')
    d3d_output = open(d3d_output_path, 'w')
    subprocess.call([self.exec_dream3d, "-p", pipeline], stdout=d3d_output, stderr=d3d_output)

    print("runD3D_mazeLocators: Runner Complete.")

    return (fpath_maze, [fpath_redDot, fpath_greenDot, fpath_blueDot])



def runD3D_maurer_filter(self, input_maze_image: np.ndarray) -> tuple:
    """
    Processor caller for preconfigured Dream3D pipeline.
    Requires: os, subprocess, json, rospy, rospkg

    :param input_maze_image:    Cropped input maze image.
    :return output_maurer_path: Absolute file path to the Maurer Filtered maze image
    """

    # Save Image to file
    fpath_input = os.path.join(self.dir_wksp, 'maze_masked.tiff')
    cv2.imwrite(fpath_input, input_maze_image)

    # Setup Dream3D Pipeline
    pipeline = os.path.join(self.dir_resources, 'pipeline_maurer_path.json')
    fpath_output = os.path.join(self.dir_wksp, 'maze_maurer_path.tiff')

    # Setup Dream3D / Simple Pipeline
    with open(pipeline, 'r') as jsonFile:
        data = json.load(jsonFile)
        data["0"]["FileName"] = fpath_input
        data["4"]["SelectedThresholds"][0]["Comparison Value"] = 0.5
        data["7"]["FileName"] = fpath_output

    with open(pipeline, 'w') as jsonFile:
        json.dump(data, jsonFile, indent=4)

    # Run Dream3D Pipeline
    d3d_output_path = os.path.join(self.dir_log, 'log_D3D_maurerPath.txt')
    d3d_output = open(d3d_output_path, 'w')
    subprocess.call([self.exec_dream3d, "-p", pipeline], stdout=d3d_output, stderr=d3d_output)

    # Read in Img Result
    output_img = cv2.imread(fpath_output, 0)

    return output_img, fpath_output