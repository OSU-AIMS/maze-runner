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

from sensor_msgs.msg import Image
from maze_runner.msg import MazeData


class VISION_PROCESSING():

    def __init__(self, pub):
        self.publish_results = pub
        self.color = Image
        self.depth = Image

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
        ## call post-processor

        ## fill in a new message based on information recieved from post-processor
        msg = MazeData
        
        ## publish message
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
    rate = rospy.Rate(1) # 1hz = 1 cycle/sec
    rospy.loginfo("Vision Post-Processing Node Started")

    pub         = rospy.Publisher("MazeData", MazeData, queue_size=3)
    sub_color   = message_filters.Subscriber("/camera/color/image_raw", Image)
    sub_depth   = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

    vp = VISION_PROCESSING(pub)

    # Exact Time Sync required. Consider using message_filters.ApproximateTimeSynchronizer in future
    ts = message_filters.TimeSynchronizer([sub_color, sub_depth], queue_size=10)
    ts.registerCallback(vp.synchronousCallback)

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