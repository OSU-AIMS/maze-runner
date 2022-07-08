#!/usr/bin/env python3
#
# Toolbox for Fiducial-based Pose estimation of a gameboard
# 

#### IMPORTS ####
from cmath import nan
import cv2
import numpy as np

from sensor_msgs.msg import CameraInfo


#### TOOLS ####
class FIDUCIAL_METHOD_ARUCO():
    """
    Toolbox for Fiducial-based Pose estimation of a gameboard
    """

    def __init__(self):

        # Marker Dectection Variables
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.aruco_param = cv2.aruco.DetectorParameters_create()
    
        # Known Markers
        self.last_known_markers = {
            '1': self.Marker(0, (nan, nan), (nan, nan, nan)),
            '2': self.Marker(0, (nan, nan), (nan, nan, nan)),
            '3': self.Marker(0, (nan, nan), (nan, nan, nan)),
            '4': self.Marker(0, (nan, nan), (nan, nan, nan))
        }

    class Marker():
        '''
        Data structure for fiducial marker information.
        '''
        def __init__(self, id: int, centroid: tuple, quant_rotation: tuple) -> None:
            # if __debug__:
            # Type Checking
            # if not isinstance(id, int): raise TypeError
            # if not isinstance(centroid, tuple): raise TypeError
            # if not isinstance(quant_rotation, tuple): raise TypeError
            # if not len(centroid) == 2: raise AssertionError
            # if not len(quant_rotation) == 3: raise AssertionError

            # Data Structure
            self.id = id
            self.centroid = centroid
            self.quant_rotation = quant_rotation

    def detect_markers(self, img: np.ndarray, camera_info: CameraInfo, marker_length: float):
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

        # Find Markers
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img_gray, self.aruco_dict, parameters = self.aruco_param)

        # Find Markers Pose
        (rvecs, tvecs, points) = cv2.aruco.estimatePoseSingleMarkers( corners, markerLength=marker_length, cameraMatrix=camera_info.k, distCoeffs=camera_info.d )

        img_poses = img

        # Parse Output
        for i in range(len(ids)):

            c = corners[i][0]
            centroid = (c[:, 0].mean(), c[:, 1].mean())

            # Debug: Draw Poses on Image
            img_poses = cv2.drawFrameAxes( img, cameraMatrix=camera_info.k, distCoeffs=camera_info.d, rvec=rvecs[i], tvec=tvecs[i], length=0.05, thickness=5 )

            print(ids[i][0].T)
            detected_marker = self.Marker(ids[i][0].T, centroid, rvecs[i])

        # Debug
        cv2.imwrite('demo_poses.png', img_poses)
            



if __name__ == '__main__':

    # Test Variables
    c = cv2.imread('/home/buynak.9/AA_DEVL/ws_mzrun/src/maze-runner/mzrun_ws/test_real2.png', 3)

    dummy_cam_info = CameraInfo
    dummy_cam_info.k = np.array([645.2972412109375, 0.0, 638.7476806640625, 0.0, 643.7479248046875, 356.8504943847656, 0.0, 0.0, 1.0]).reshape((3,3))
    dummy_cam_info.d = np.array([-0.05540444329380989, 0.0635504275560379, -0.0007356749847531319, 0.00030453770887106657, -0.020576464012265205])

    # Test
    fma = FIDUCIAL_METHOD_ARUCO()
    fma.detect_markers(c, dummy_cam_info, 0.02)
