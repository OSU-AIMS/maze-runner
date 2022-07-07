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


import message_filters


from maze_runner.msg import MazeData
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from cv_bridge import CvBridge, CvBridgeError


######################
## Class Controller ##
######################

class VisionProcessingControl(Node, MazeVision):
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

        # Setup Pub/Sub
        self.pub_maze_data = self.create_publisher(MazeData, "MazeData", qos_profile = 5)
        sub_color = message_filters.Subscriber(self, Image, "/camera/color/image_raw")
        sub_depth = message_filters.Subscriber(self, Image, "/camera/depth/image_rect_raw")

        # Setup TimeSychronizer
        self.ts = message_filters.TimeSynchronizer([sub_color, sub_depth], queue_size=10)
        self.ts.registerCallback(self._synchronous_callback)

        # self.get_logger().info("Waiting for first message from camera...")
        # self.get_logger().info("/camera/color/image_raw", Image)

        # Spin Cycle Controller
        self.get_logger().info('Initialized node cycle control')
        self.create_timer(1.0/cycle_freq, self._pub_results)  # 0.5 hz = 2 sec/cycle  

        # Initialize Variables
        self.color = Image
        self.depth = Image
        self.seq_counter = 0

        self.get_logger().info("Vision Post-Processing Node Started")

    def _synchronous_callback(self, data_color, data_depth) -> None:
        """
            Called by Subscriber every time message recieved stores latest set of synchronous data.
        """
        self.get_logger().warn("Test: Sychronous Callback Recieved data")
        self.color = data_color
        self.depth = data_depth

    def _pub_results(self) -> None:
        self.get_logger().warn("Test: Cycled Publisher")

    def _pub_results_real(self) -> None:
        """
            Called by rclpy timer defined external to node. 
            Calls vision processing function using latest set of synchronous data and returns results.
            Publishes results. 
        """
        # Process Timer ~ Start
        start = self.get_clock.now()
        self.seq_counter+=1


        # Process vision
        # Process Input Data from ROS message to .Mat object
        # TODO: add check to ensure only recent messages are being processed
        bridge = CvBridge()
        img_color = bridge.imgmsg_to_cv2(self.color, "bgr8")
        img_depth = bridge.imgmsg_to_cv2(self.depth, "passthrough")

        feat = self.vision_runner(img_color, img_depth)


        # TODO: post-process pose array for path
        # found_path = feat[3]
        found_path =[Pose]

        ## Build message
        msg = MazeData
        h = Header

        h.stamp     = self.get_clock.now()
        h.seq       = self.seq_counter
        h.frame_id  = self.color.header.frame_id

        msg.header           = h
        msg.scale            = feat[0]                                          # TODO: Assertion on data type
        msg.projected_origin = Pose2D(feat[1][0], feat[1][1], feat[1][2])

        p = Pose
        p.position    = Point(feat[2][0], feat[2][1], feat[2][2])
        p.orientation = Quaternion(feat[2][3], feat[2][4], feat[2][5], feat[2][6])
        msg.pose_relative_2_camera = p

        msg.path = found_path

        # Process Timer ~ End
        self.get_logger.info("Vision Post-Processer took: " + str(self.get_clock.now() - start) + " seconds.")


        ## Publish message
        # self._pub_maze_data.publish(msg)



##########
## Main ##
##########

def main(args=None):

    rclpy.init(args=args)

    # Init Node & Spin
    vp = VisionProcessingControl()
    rclpy.spin(vp)

    # Cleanup
    vp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()

