#!/usr/bin/env python3
#
# Python module
# 


import os
import cv2
import numpy as np
from ament_index_python.packages import get_package_prefix

from maze_solver.solve import MazeSolver, SolverFactory

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
    def __init__(self, set_debug=False) -> None:
        self.debug = set_debug

        # Directories and Filepaths
        self.dir_pkg     = get_package_prefix('maze_vision')
        self.dir_wksp    = os.path.join(self.dir_pkg, 'mzrun_ws')  # active workspace during runtime. holds temporary files.
        self.dir_share   = os.path.join(self.dir_pkg, 'share', 'maze_vision')
        self.dir_log     = self.dir_wksp
        self.dir_resources = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources')
        self.exec_dream3d = "/usr/local/programs/DREAM3D/bin/PipelineRunner"

        # Fiducial Method
        self.fma = self.FIDUCIAL_METHOD_ARUCO(marker_length = 0.02, set_debug=self.debug)

        # Maze Solver
        self.sf = SolverFactory()
        _, self.solve_algorithm = self.sf.createsolver(self.sf.Choices[2])

        # Setup Runtime Workspace
        if not os.path.exists(self.dir_wksp):
            os.makedirs(self.dir_wksp)
            print("[info] Created runtime workspace.")
        return

    '''Imported Methods'''
    from ._FIDUCIAL_METHOD_ARUCO import FIDUCIAL_METHOD_ARUCO
    from ._FIDUCIAL_METHOD_RGB_DOTS import FIDUCIAL_METHOD_RGB_DOTS
    from ._PRCS_DEPTH import smooth_depth_map
    from ._PRCS_PATH_SOLVER import clean_maurer_path, call_path_solver
    from ._RUN_D3D import runD3D_maze_locators, runD3D_maurer_filter


    '''Runner'''
    def vision_runner(self, image_color: np.ndarray, image_depth: np.ndarray, camera_info: CameraInfo) -> list:

        # Reference Variables
        dir_wksp = os.path.join(get_package_prefix('maze_vision'), 'mzrun_ws')

        fpath_color = os.path.join(dir_wksp,'color.tiff')
        fpath_depth = os.path.join(dir_wksp,'depth.tiff')

        cv2.imwrite(fpath_color, image_color)
        cv2.imwrite(fpath_depth, image_depth)
        
        # Runner
        if not self.fma.find_board(image_color, camera_info):

            # Retrieve Board Results
            img_cropped_maze = self.fma.get_cropped_maze()

            # Clean Masked Maze Surface
            ret, thresh = cv2.threshold(img_cropped_maze, 100, 255, cv2.THRESH_BINARY)
            # kernel = np.ones((3,3), np.uint8)
            # img_cropped_maze_binary = cv2.erode(thresh, kernel)

            # Simplify & Clean Cropped Maze Image
            result_img, result_fpath = self.runD3D_maurer_filter(thresh)
            maurer_image = self.clean_maurer_path(result_img)

            # Run Python Maze Solver on Cleaned Image
            solved_path_list, solved_img = MazeSolver(maurer_image, self.solve_algorithm)
            # solved_csv_filepath, solved_image_filepath = self.call_path_solver(maurer_image)

            # Debug
            if self.debug:
                cv2.imwrite('debug_step5_cv2_cleanup.tiff', thresh)
                cv2.imwrite('debug_step6_maurer_cleaned.tiff', maurer_image)
                cv2.imwrite('debug_step7_solved_path.tiff', solved_img)

            # # Depth
            # # temporary, grab depth at origin and assume flat.
            # # TODO: new method to get depth at every waypoing along path
            # depth_features = self.smooth_depth_map(image_depth)
            # depth_origin = depth_features[features.mazeCentroid[0], features.mazeCentroid[1]]


            # # Assemble Pose relative to camera link
            # # maze_pose_relative_2_camera = [x,y,z,x,y,z,w]
            # rot3d_tf = np.identity(4)
            # rot3d_tf[:-1,:-1] = features.rotMatrix.astype("float64")
            # rot3d_tf[:-1, 3] = np.array([features.mazeCentroid[0], features.mazeCentroid[1], depth_origin])

            # rot3d_q = tf.transformations.quaternion_from_matrix(rot3d_tf)
            # rot3d_v = tf.transformations.translation_from_matrix(rot3d_tf)
            # maze_pose_relative_2_camera = np.hstack((rot3d_v, rot3d_q))

            # Assemble Path
            path = []

            # Return Results
            # result = [features.scale, projected_2d_pose, maze_pose_relative_2_camera, path]

            return [], solved_img.astype('uint8')
        
        else:
            return 1



##########################
## FOR DEVELOPMENT ONLY ##
##########################

if __name__ == '__main__':

    from maze_vision import MazeVision

    c = cv2.imread('/home/buynak.9/AA_DEVL/ws_mzrun/src/maze-runner/mzrun_ws/test_real2.png', 3)
    d = cv2.imread('/home/buynak.9/AA_DEVL/ws_mzrun/src/maze-runner/mzrun_ws/depth.tiff', -1)

    dummy_cam_info = CameraInfo
    dummy_cam_info.k = np.array([645.2972412109375, 0.0, 638.7476806640625, 0.0, 643.7479248046875, 356.8504943847656, 0.0, 0.0, 1.0]).reshape((3,3))
    dummy_cam_info.d = np.array([-0.05540444329380989, 0.0635504275560379, -0.0007356749847531319, 0.00030453770887106657, -0.020576464012265205])

    mv = MazeVision()
    mv.vision_runner(c,d,dummy_cam_info)
    