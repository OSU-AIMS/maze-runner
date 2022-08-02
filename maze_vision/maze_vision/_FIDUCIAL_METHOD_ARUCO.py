#!/usr/bin/env python3
#
# Toolbox for Fiducial-based Pose estimation of a gameboard
# 

#### IMPORTS ####
import cv2
import numpy as np

from sensor_msgs.msg import CameraInfo


#### TOOLS ####
class FIDUCIAL_METHOD_ARUCO():
    """
    Toolbox for Fiducial-based Pose estimation of a gameboard
    """

    def __init__(self, marker_length, set_debug = False):
        '''
        marker_length: Real world edge-length of the square fiducial markers. 
        '''
        self.debug = set_debug

        # Marker Dectection Variables
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.aruco_param = cv2.aruco.DetectorParameters_create()
    
        # Init Varaibles
        self._img_poses_annotated = np.zeros((720,1280))

        # Known Markers
        self.marker_length = marker_length
        self.last_known_markers = { }


    ''' DATA STRUCTURE '''
    class Marker():
        '''
        Data structure for fiducial marker information.
        '''
        def __init__(self, id: int, centroid: tuple, axis_angle_rotation: tuple) -> None:
            # if __debug__:
            # Type Checking
            # if not isinstance(id, int): raise TypeError
            # if not isinstance(centroid, tuple): raise TypeError
            # if not isinstance(axis_angle_rotation, tuple): raise TypeError
            # if not len(centroid) == 2: raise AssertionError
            # if not len(axis_angle_rotation) == 3: raise AssertionError

            # Data Structure
            self.id = id
            self.centroid = centroid
            self.rvec = axis_angle_rotation


    ''' PUBLIC METHODS '''
    def find_board(self, color_image: np.ndarray, cam_info: CameraInfo) -> int:
        '''
            Tool wrapper.
        '''

        r1 = self._detect_markers(color_image, camera_info=cam_info)
        if not r1: 
            r2 = self._calc_board_2D_rotmat()
        else:
            return r1
        if not ( r1 or r2 ): 
            r3 = self._maze_mask(color_image)
        else:
            return r2
        if r1 or r2 or r3:
            return r3

    def get_cropped_maze(self) -> np.ndarray:
        return self.img_cropped_maze

    def get_img_annotated_w_markers(self) -> np.ndarray:
        return self._img_poses_annotated

    def get_maze_origin_transform_matrix(self) -> np.ndarray:
        """ Returns origin of maze in the original color frames pixel coordinates. Origin centered on Marker ID #1 """
        tf_ = np.identity(4)
        tf_[:-1, :-1] = self.board_2d_rotmatrix
        tf_[0, -1] = self.last_known_markers[1].centroid[0]
        tf_[1, -1] = self.last_known_markers[1].centroid[1]
        return tf_

    ''' PROTECTED METHODS '''
    def _detect_markers(self, img: np.ndarray, camera_info: CameraInfo):
        '''
        Find Fiducial markers and report location.
        img: grayscale image containing markers to find
        camera_info: ROS CameraInfo message containing camera intrinsics
        marker_length: Real world side-length of the square fiducial markers in METERS.
        '''
        # Ensure grayscale input
        if not len(img.shape) == 2:
            print(" > INFO: Input is not grayscale. Converting now, then proceeding.")
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            img_gray = img

        # Find Markers & Pose
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img_gray, self.aruco_dict, parameters = self.aruco_param)

        # Clear previously found markers
        self.last_known_markers.clear()

        # Parse Output
        if ids is not None:
            # Calculate Poses of Detected Markers
            (rvecs, tvecs, points) = cv2.aruco.estimatePoseSingleMarkers( corners, markerLength=self.marker_length, cameraMatrix=np.asarray(camera_info.k).reshape((3,3)), distCoeffs=np.asarray(camera_info.d) )

            # Process Identified Markers
            img_poses = img.copy()
            detected_markers = {}
            for i in range(len(ids)):

                c = corners[i][0]
                centroid = (c[:, 0].mean(), c[:, 1].mean())

                detected_markers[ids[i][0].T] = self.Marker(ids[i][0].T, centroid, rvecs[i])

                # Debug: Draw Poses on Image
                cv2.drawFrameAxes( img_poses, cameraMatrix=np.asarray(camera_info.k).reshape((3,3)), distCoeffs=np.asarray(camera_info.d), rvec=rvecs[i], tvec=tvecs[i], length=0.05, thickness=5 )

            # Debug
            self._img_poses_annotated = img_poses
            if self.debug: cv2.imwrite('debug_step1_poses_annotated.tiff', img_poses)

            # Return detected markers
            self.last_known_markers = detected_markers
            return 0
        else:
            print("Something went wrong parsing detected Fiducial Markers. Were any markers detected?")
            return 1

    def _calc_board_2D_rotmat(self) -> np.ndarray:
        '''
        Method: All four markers required to determine board pose
        Assumptions:
            - Board surface is planar and closely perpendicular to camera axis.
            - Board corner number convention is met (see `c_topleft` definitions in method source code)
        '''
        # Data Validation
        if not len(self.last_known_markers) == 4: 
            print('All four fiducial markers required to calculate board info')
            return 1
        
        # Convention:   Right-Hand-Rule
        #   Origin  :   @ TOP.LEFT
        #   X-Vector:   TOP.LEFT -> TOP.RIGHT
        #   Y-Vector:   TOP.LEFT -> TOP.RIGHT (but inverted across x-axis)

        c_topleft  = self.last_known_markers[1].centroid
        c_topright = self.last_known_markers[2].centroid
        c_botleft  = self.last_known_markers[3].centroid
        c_botright = self.last_known_markers[4].centroid

        # Find Maze Axis Vectors
        axis_x = np.subtract(c_topright, c_topleft, where="array_like")
        axis_y = np.subtract(c_botleft, c_topleft, where="array_like")

        axis_x = axis_x / np.linalg.norm(axis_x, 2)
        axis_y = axis_y / np.linalg.norm(axis_y, 2)

        axis_x = np.append(axis_x, 0)
        axis_y = np.append(axis_y, 0)

        # Check if Vectors are sufficiently square to each other
        dot_prod = np.dot(axis_x, axis_y)
        if dot_prod < np.cos(np.deg2rad(45)):
            axis_z = np.cross(axis_x, axis_y)
            axis_z = axis_z / np.linalg.norm(axis_z, 2)
        else:
            print(" > WARN: X & Y axis insufficiently square. Failed to assemble rotation matrix.")
            return 2
    
        # Package into Rotation Matrix
        # https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Formalism_alternatives
        rot_matrix = np.stack([axis_x, axis_y, axis_z], axis=1)
        
        # Return
        self.board_2d_rotmatrix = rot_matrix
        return 0

    def _maze_mask(self, img) -> np.ndarray:
        """
        Generate a mask of region confined by a quadrilateral constrained by the fiducial markers
        :param dots: Input Dictionary with three dot keys.
        :param img: cv2 mat 8-bit image (0-255) containing the maze
        :return: exit status
        """

        # Check Input
        assert self.board_2d_rotmatrix.shape == (3, 3), "Assert: Rotation Matrix must be size (3,3)."
        assert np.isnan(self.board_2d_rotmatrix).any() == False, "Assert: Rotation Matrix must not contan NaN values."
      
        # Convert to Grayscale
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find Mask Corners
        c_topleft  = self.last_known_markers[1].centroid
        c_topright = self.last_known_markers[2].centroid
        c_botleft  = self.last_known_markers[3].centroid
        c_botright = self.last_known_markers[4].centroid
        print(" > INFO: Four Corners Identified @ :", c_topleft, c_topright, c_botleft, c_botright)

        # Build Mask
        height    = img_gray.shape[0]
        width     = img_gray.shape[1]
        frame     = np.zeros((height, width), dtype=np.uint8)
        centroids = np.stack((c_topleft, c_topright, c_botright, c_botleft), axis=0)
        mask = cv2.fillPoly(frame, pts=[centroids.astype('uint64')], color=(255))

        img_mask = cv2.bitwise_and(img_gray, img_gray, mask = mask)

        # Export Masked Image
        if self.debug: cv2.imwrite('debug_step2_masked.tiff', img_mask)

        ## Rotate Image about Center of Maze
        center_x = np.mean(centroids[:,0], dtype=np.uint64)
        center_y = np.mean(centroids[:,1], dtype=np.uint64)

        # Calculate Rotation Angle
        rotmat_inv = np.linalg.inv(self.board_2d_rotmatrix).astype(np.float64)[:-1, :]
        angle = -np.arctan2(rotmat_inv[1,0], rotmat_inv[0,0])
        counter_rotate = cv2.getRotationMatrix2D((center_x, center_y), np.degrees(angle), scale=1.0)

        edge_length_px = np.linalg.norm(np.subtract(c_topright, c_topleft, where="array_like"), 2).astype('uint64')
        maze_shape = ( edge_length_px, edge_length_px)

        img_masked = cv2.warpAffine(img_mask, counter_rotate, dsize=(img_mask.shape[1], img_mask.shape[0]), flags=cv2.INTER_NEAREST)

        # Debug Export 
        if self.debug: cv2.imwrite('debug_step3_masked_rotated.tiff', img_masked)


        ## Crop Image
        mask_non_black = img_masked > 0
        coord_non_black = np.argwhere(mask_non_black)

        x0, y0 = coord_non_black[:,:2].min(axis=0)
        x1, y1 = coord_non_black[:,:2].max(axis=0) + 1

        img_cropped = img_masked[x0:x1, y0:y1]
        if self.debug: cv2.imwrite('debug_step4_masked_rotated_cropped.tiff', img_cropped)

        # Return
        self.img_cropped_maze = img_cropped
        return 0


''' TESTING '''
if __name__ == '__main__':

    # Test Variables
    c = cv2.imread('/home/buynak.9/AA_DEVL/ws_mzrun/src/maze-runner/mzrun_ws/test_real2.png', 3)

    dummy_cam_info = CameraInfo
    dummy_cam_info.k = np.array([645.2972412109375, 0.0, 638.7476806640625, 0.0, 643.7479248046875, 356.8504943847656, 0.0, 0.0, 1.0]).reshape((3,3))
    dummy_cam_info.d = np.array([-0.05540444329380989, 0.0635504275560379, -0.0007356749847531319, 0.00030453770887106657, -0.020576464012265205])

    # Test
    fma = FIDUCIAL_METHOD_ARUCO(marker_length = 0.02, set_debug=True)
    fma.find_board(c, dummy_cam_info)

    img = fma.get_cropped_maze()