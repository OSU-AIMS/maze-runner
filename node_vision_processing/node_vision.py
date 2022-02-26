#!/usr/bin/env python
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2022, The Ohio State University
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
#
# Author: acbuynak
#
# Node: vision_processing

import rospy
import message_filters

import node_vision_support

from maze_runner.msg import MazeData
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose

from cv_bridge import CvBridge, CvBridgeError


class VISION_PROCESSING_CONTROLLER():

    def __init__(self, pub):
        self.publish_results = pub
        self.color = Image
        self.depth = Image
        self.seq_counter = 0

    def testColorCallback(self, data_color):
        """
            TEMPORARY FUNCTION.
            Used for testing the color image input ONLY.
        """
        self.color = data_color

    def synchronousCallback(self, data_color, data_depth):
        """
            Called by Subscriber every time message recieved stores latest set of synchronous data.
        """
        self.color = data_color
        self.depth = data_depth

    def pubResults(self):
        """
            Called every rospy.Rate(#) hz. 
            Calls vision processing function using latest set of synchronous data and returns results.
            Publishes results. 
        """
        # Process Timer ~ Start
        start = rospy.get_time()
        self.seq_counter+=1


        # Process vision
        # Process Input Data from ROS message to .Mat object
        # TODO: add check to ensure only recent messages are being processed
        bridge = CvBridge()
        img_color = bridge.imgmsg_to_cv2(self.color, "bgr8")
        img_depth = bridge.imgmsg_to_cv2(self.depth, "passthrough") 

        feat = node_vision_support.VISION_PROCESSOR(img_color, img_depth)


        # TODO: post-process pose array for path
        found_path = feat[3]

        ## Build message
        msg = MazeData
        h = Header

        h.stamp     = rospy.Time.now()
        h.seq       = self.seq_counter
        h.frame_id  = self.color.header.frame_id

        msg.header           = h
        msg.scale            = feat[0]                                          # TODO: Assertion on data type
        msg.projected_origin = Pose2D(feat[1][0], feat[1][1], feat[1][2])
        msg.pose_relative_2_camera.Point = Point(feat[2][0], feat[2][1], feat[2][2])
        msg.pose_relative_2_camera.Quaternion = Quaternion(feat[2][3], feat[2][4], feat[2][5], feat[2][6])
        msg.path = found_path

        # Process Timer ~ End
        rospy.loginfo("Vision Post-Processer took: " + str(rospy.get_time() - start) + " seconds.")


        ## Publish message
        self.publish_results.publish(msg)


def main():
    """
        Initialize ROS node and set publishing rate.
        Setup publisher & subscriber connections.
        Using `message_filters` package to time synchronize data recieved from camera.
        Process data recieved.
        Publish to custom ROS message. 
    """

    rospy.init_node('vision_processing', anonymous=False)
    rate = rospy.Rate(0.5) # 1hz = 1 cycle/sec
    rospy.loginfo("Vision Post-Processing Node Started")

    pub         = rospy.Publisher("MazeData", MazeData, queue_size=3)
    sub_color   = message_filters.Subscriber("/camera/color/image_raw", Image)
    sub_depth   = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

    vp = VISION_PROCESSING_CONTROLLER(pub)

    # Exact Time Sync required. Consider using message_filters.ApproximateTimeSynchronizer in future
    ts = message_filters.TimeSynchronizer([sub_color, sub_depth], queue_size=10)
    ts.registerCallback(vp.synchronousCallback)

    rospy.loginfo("Waiting for first message from camera...")
    rospy.wait_for_message("/camera/color/image_raw", Image)

    while not rospy.is_shutdown():
        vp.pubResults()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        exit()
    except KeyboardInterrupt:
        exit()

