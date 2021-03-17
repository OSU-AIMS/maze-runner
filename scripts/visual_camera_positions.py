#!/usr/bin/env python

#####################################################
##          RViz. Pose Array Publisher             ##
#####################################################
# Software License Agreement (BSD License)          #
# Author: Adam Buynak                               #
#####################################################

## IMPORTS ##
import rospy
import numpy as np
import geometry_msgs.msg

## SUPPORT FUNCTIONS ##


## NODES ##
def node_cameraPoseArray(inputArray):
    """ Publish and Latch a Pose Array to a rostopic. """
    
    # Imports
    import rospy
    from geometry_msgs.msg import PoseArray

    # Config node
    pub = rospy.Publisher('cameraPoseArray', PoseArray, queue_size=10) #TODO: couldn't get latch=True to work. Looping instead
    rospy.init_node('cameraPoseArray', anonymous=False)
    rate = rospy.Rate(1) # 10hz

    message = geometry_msgs.msg.PoseArray()
    message.header.frame_id = 'base_link'
    message.poses = inputArray
    
    # Publish node
    while not rospy.is_shutdown():
        #rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()




## MAIN CODE ##
def main():

    import rospy
    import numpy as np
    import geometry_msgs.msg
    from rospy_message_converter import json_message_converter


    # Find mzrun_ws
    dir_mzrun = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    mzrun_ws = dir_mzrun + '/mzrun_ws'

    with open(mzrun_ws + '/camera_poses.json') as file:
         = json_message_converter.convert_json_to_ros_message(file)


    # Try launching ros node
    try:
        node_cameraPoseArray(pose_geom)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
