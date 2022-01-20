#! /usr/bin/python
#
# Support file for node: vision_processing
# 

def cleanMaurerPath(maurer_image_filepath):
    """
    Call Dream3D Maurer Filter Pipeline. Post-process generate image.

    :param maurer_image_filepath: Absolute file path to the D3D Maurer filtered maze
    :return: Saves new Image (maze_maurer_path.tiff)
    """

    import cv2
    import numpy as np

    # Load Maurer Filtered Image
    img = cv2.imread(maurer_image_filepath, 0)

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



def callPathSolver(filepath_maurer_path):
    """
    Wapper around Python Maze Solver script.

    :param path_image_cropped: Absolute file path to the cropped maze
    :return: Filename (less extension)  (maze_maurer_path.tiff)
    """
    #todo: using this as a wrapper. In future, covert solver to a python class and import

    import os
    import subprocess
    import rospy

    # Directories and Filepaths
    rospack         = rospkg.RosPack()
    dir_pkg         = rospack.get_path('maze_runner')
    dir_wksp        = os.path.join(dir_pkg, 'mzrun_ws')
    dir_log         = dir_wksp                          #os.environ.get('ROS_LOG_DIR')
    solver          = os.path.join(dir_pkg, 'include/mazesolving/solve.py')
    output_filename = os.path.join(dir_wksp, 'path_solved')

    # Logs
    solver_output_path = os.path.join(dir_log, 'log_pathSolver.txt')
    solver_output = open(solver_output_path, 'w') # Workaround to supress output.

    # Run Solver
    subprocess.call(["python", solver, filepath_maurer_path, output_filename], stdout=solver_output, stderr=solver_output)

    return (str(output_filename) + '.csv', str(output_filename) + '.tiff')
    