#!/usr/bin/env python3
#
# Python module
# 


import os
import cv2
import numpy as np
from ament_index_python.packages import get_package_prefix

from sensor_msgs.msg import CameraInfo

# import tf_transformations


################
## Main Class ##
################

class MazeVision():
    '''
        Maze Runner projects vision processing toolbox. 
    '''

    '''Setup'''
    def __init__(self) -> None:

        # Directories and Filepaths
        self.dir_pkg     = get_package_prefix('maze_runner')
        self.dir_wksp    = os.path.join(self.dir_pkg, 'mzrun_ws')  # active workspace during runtime. holds temporary files.
        self.dir_share   = os.path.join(self.dir_pkg, 'share', 'maze_runner')
        self.dir_log     = self.dir_wksp
        self.dir_resources = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources')
        self.exec_dream3d = "/usr/local/programs/DREAM3D/bin/PipelineRunner"

        # Setup Runtime Workspace
        if not os.path.exists(self.dir_wksp):
            os.makedirs(self.dir_wksp)
            print("[info] Created runtime workspace.")
        return

    '''Imported Methods'''
    from ._FIDUCIAL_METHOD_RGB_DOTS import FIDUCIAL_METHOD_RGB_DOTS
    from ._PRCS_DEPTH import smooth_depth_map
    from ._PRCS_PATH_SOLVER import clean_maurer_path, call_path_solver
    from ._RUN_D3D import runD3D_maze_locators, runD3D_maurer_filter


    '''Runner'''
    def vision_runner(self, image_color: np.ndarray, image_depth: np.ndarray, camera_info: CameraInfo) -> list:

        # Reference Variables
        dir_wksp = os.path.join(get_package_prefix('maze_runner'), 'mzrun_ws')

        fpath_color = os.path.join(dir_wksp,'color.tiff')
        fpath_depth = os.path.join(dir_wksp,'depth.tiff')

        cv2.imwrite(fpath_color, image_color)
        cv2.imwrite(fpath_depth, image_depth)


        if False:
            # Run Pose Processor
            dot_names = ['redDot', 'greenDot', 'blueDot']
            fpath_masked_maze, fpaths_dot_feature = self.runD3D_maze_locators(fpath_color, dot_names)

            # Post Process Results
            maze_size = [0.18, 0.18] # Realworld Measurement (in meters)
            features = self.FIDUCIAL_METHOD_RGB_DOTS(dot_names, fpaths_dot_feature, fpath_masked_maze, maze_size)
            input_maze_image_filepath = features.exportResults()

            # Assemble 2D Pose
            # projected_2d_pose = [x,y,theta]
            projected_2d_pose = [features.mazeCentroid[0], features.mazeCentroid[1], features.rotAngle]

        

        # Simplify & Clean Cropped Maze Image
        raw_filter_result       = self.runD3D_maurer_filter(input_maze_image_filepath)
        maurer_image_filepath   = self.clean_maurer_path(raw_filter_result)


        # Run Python Maze Solver on Cleaned Image
        solved_csv_filepath, solved_image_filepath = self.call_path_solver(maurer_image_filepath)


        # Depth
        # temporary, grab depth at origin and assume flat.
        # TODO: new method to get depth at every waypoing along path
        depth_features = self.smooth_depth_map(image_depth)
        depth_origin = depth_features[features.mazeCentroid[0], features.mazeCentroid[1]]


        # Assemble Pose relative to camera link
        # maze_pose_relative_2_camera = [x,y,z,x,y,z,w]
        rot3d_tf = np.identity(4)
        rot3d_tf[:-1,:-1] = features.rotMatrix.astype("float64")
        rot3d_tf[:-1, 3] = np.array([features.mazeCentroid[0], features.mazeCentroid[1], depth_origin])

        rot3d_q = tf.transformations.quaternion_from_matrix(rot3d_tf)
        rot3d_v = tf.transformations.translation_from_matrix(rot3d_tf)
        maze_pose_relative_2_camera = np.hstack((rot3d_v, rot3d_q))

        # Assemble Path
        path = []

        # Return Results
        result = [features.scale, projected_2d_pose, maze_pose_relative_2_camera, path]

        return result



##########################
## FOR DEVELOPMENT ONLY ##
##########################

if __name__ == '__main__':

    from maze_vision import MazeVision

    c = cv2.imread('/home/buynak.9/AA_DEVL/ws_mzrun/src/maze-runner/mzrun_ws/color.tiff', 3)
    d = cv2.imread('/home/buynak.9/AA_DEVL/ws_mzrun/src/maze-runner/mzrun_ws/depth.tiff', -1)

    mv = MazeVision()
    mv.vision_runner(c,d)
    