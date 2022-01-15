#!/usr/bin/env python
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2022, The Ohio State University
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
#
# Author: acbuynak
#
# Node: maze_vision_processing


import rospy
import numpy as np


def main():

	rospy.init_node('maze_vision_processing', anonymous=False)
    rospy.loginfo("Vision Post-Processing Node Started")


    rospy.Subscriber("/camera/color/image_raw", Image, callback_frames)
    rospy.Subscriber("camera/aligned_depth_to_color/image_raw", Image, callback_frames)


    pub_boardPOS = rospy.Publisher("maze_info", Image, queue_size=3)



	try:
		rospy.spin()
    except rospy.ROSInterruptException:
        exit()
    except KeyboardInterrupt:
        exit()


if __name__ == '__main__':
    main()