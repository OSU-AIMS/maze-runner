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
class node_cameraPoseArray(object):
    """ Publish and Latch a Pose Array to a rostopic. """

    def __init__(self, inputArray):
        
        # Imports
        import rospy
        from geometry_msgs.msg import PoseArray

        # Config node
        self.pub = rospy.Publisher('camera_pose_array', PoseArray, queue_size=10, latch=True) #TODO: couldn't get latch=True to work. Looping instead
        rospy.init_node('camera_pose_array', anonymous=False)
        self.rate = rospy.Rate(1) # number of times per second

        self.message = geometry_msgs.msg.PoseArray()
        self.message.header.frame_id = 'origin'
        self.message.poses = inputArray


def update_pose_array(json_file):
    """
    Opens and reads in a list of poses from json
    :param json_file: Input absolute filepath
    :return                 List of poses in ROS message Format
    """

    import json
    from rospy_message_converter import json_message_converter
    import geometry_msgs.msg

    with open(json_file) as file:
        data = json.load(file)

    poselist_ros_format = []
    for i in data['poses']:
        poselist_ros_format.append(json_message_converter.convert_json_to_ros_message('geometry_msgs/Pose', i))

    return poselist_ros_format




## MAIN CODE ##
def main():

    from os import path
    from time import sleep

    import rospy
    import numpy as np


    # Find mzrun_ws
    #dir_mzrun = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    json_file_name = '/camera_poses.json'
    mzrun_ws = '//path/new'
    input_json_file = mzrun_ws + json_file_name


    # Check for File
    while True:
        #todo: consider switching this to 'threading.timer' to reduce processor load
        if path.exists(input_json_file):
            print("<< Found input camera positions file.")
            break
        sleep(5.0)


    # Pre-Loop Setup
    poselist_ros_format = update_pose_array(input_json_file)
    # node = node_cameraPoseArray(poselist_ros_format)
    # node.pub.publish(node.message)

    # Try launching ros node
    try:
        node = node_cameraPoseArray(poselist_ros_format)
        node.rate = rospy.Rate(1) # number of times per second


        # Read File in loop until terminated.
        while not rospy.is_shutdown():
            poselist_ros_format = update_pose_array(input_json_file)
            node.message.poses = poselist_ros_format

            node.rate.sleep()

            rospy.loginfo(node.message)
            node.pub.publish(node.message)
            print("<<< Updated Camera Pose Array Node")

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
