#!/usr/bin/env python3
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2022, The Ohio State University
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
#
# Author: acbuynak
#
# Node: vision_processing


#############
## Imports ##
#############

import rclpy
from rclpy.node import Node
from maze_vision import MazeVision

import numpy as np

import message_filters

from maze_msgs.msg import MazeData
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from cv_bridge import CvBridge, CvBridgeError


######################
## Class Controller ##
######################

class VisionProcessingControl(Node):
    '''
        1 - Initialize ROS node and set publishing rate.
        2 - Setup publisher & subscriber connections.
        3 - Using `message_filters` package to time synchronize data recieved from camera.
        4 - Process data recieved.
        5 - Publish to custom ROS message. 
        
        cycle_freq float: Vision processing cycle frequency in Hz. Defaults to 0.5
    '''

    def __init__(self, cycle_freq=0.5) -> None:

        # Setup Node
        super().__init__('vision_processing')

        # Setup Pub/Sub & TimeSychronizer
        self.pub_maze_data = self.create_publisher(MazeData, "maze_data", qos_profile = 5)
        self.pub_solved_maze = self.create_publisher(Image, "solved_maze", qos_profile = 5)
        self.sub_intrinsics  = self.create_subscription(CameraInfo, "/rs1/color/camera_info", self._callback_intrinsics, 2)
        sub_color = message_filters.Subscriber(self, Image, "/rs1/color/image_raw")
        sub_depth = message_filters.Subscriber(self, Image, "/rs1/depth/image_rect_raw")

        self.ts = message_filters.TimeSynchronizer([sub_color, sub_depth], queue_size=10)
        self.ts.registerCallback(self._synchronous_callback)

        # Spin Cycle Controller
        self.get_logger().info('Initialized node cycle control')
        self.create_timer(1.0/cycle_freq, self._pub_results_real)  # 0.5 hz = 2 sec/cycle  

        # Initialize Variables
        self.camera_info = None
        self.color = None
        self.depth = None
        self.seq_counter = 0

        # Initialize Vision Processing Tool
        self.mviz = MazeVision(set_debug=False)
        self.cvbridge = CvBridge()

        # Report
        self.get_logger().info("Vision Post-Processing Node Started")

    def _synchronous_callback(self, data_color, data_depth) -> None:
        """
            Called by Subscriber every time message recieved stores latest set of synchronous data.
        """
        self.get_logger().debug("Test: Sychronous Callback Recieved data")
        self.color = data_color
        self.depth = data_depth

    def _callback_intrinsics(self, data) -> None:
        self.camera_info = data

    def _pub_results(self) -> None:
        self.get_logger().warn("Test: Cycled Publisher")

    def _pub_results_real(self) -> None:
        """
            Called by rclpy timer defined external to node. 
            Calls vision processing function using latest set of synchronous data and returns results.
            Publishes results. 
        """

        if self.color == None or self.depth == None or self.camera_info == None:
            self.get_logger().info("Waiting for first message from camera...")
            pass
        else:
            # Process Timer ~ Start
            start = self.get_clock().now()
            self.seq_counter+=1


            # Process vision
            # Process Input Data from ROS message to .Mat object
            # TODO: add check to ensure only recent messages are being processed
            c = self.cvbridge.imgmsg_to_cv2(self.color, 'bgr8')
            d = self.cvbridge.imgmsg_to_cv2(self.depth, "passthrough")

            # Runner
            runner_feedback = self.mviz.vision_runner(image_color=c, image_depth=d, camera_info=self.camera_info)

            if runner_feedback == 1:
                self.get_logger().warn("Maze Vision unable to process inputs.")
            else:
                vision_data, solved_maze_img = runner_feedback


                # TODO: post-process pose array for path
                # found_path = vision_data[3]
                found_path =[Pose]

                ## Build message
                h = Header()

                h.stamp     = self.get_clock().now().to_msg()
                h.frame_id  = self.color.header.frame_id

                # Build Maze Feature Data Message
                msg = MazeData
                msg.header           = h
                # msg.scale            = vision_data[0] # TODO: Assertion on data type
                # msg.projected_origin = Pose2D(vision_data[1][0], vision_data[1][1], vision_data[1][2])

                # p = Pose
                # p.position    = Point(vision_data[2][0], vision_data[2][1], vision_data[2][2])
                # p.orientation = Quaternion(vision_data[2][3], vision_data[2][4], vision_data[2][5], vision_data[2][6])
                # msg.pose_relative_2_camera = p

                # msg.path = found_path

                # Build Solved Maze Image Message
                msg_img = self.cvbridge.cv2_to_imgmsg(solved_maze_img, 'rgb8')
                msg_img.header = h

                ## Publish message
                # self.pub_maze_data.publish(msg)
                self.pub_solved_maze.publish(msg_img)

            # Process Timer ~ End
            self.get_logger().info("Vision Post-Processer took: " + str(self.get_clock().now() - start) + " seconds.")


##########
## Main ##
##########

def main(args=None):

    rclpy.init(args=args)

    # Init Node & Spin
    vp = VisionProcessingControl(3)
    rclpy.spin(vp)

    # Cleanup
    vp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()

