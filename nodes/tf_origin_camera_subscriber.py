#!/usr/bin/env python

import rospy
import csv
import sys
from geometry_msgs.msg import TransformStamped

def callback(data):
    # Export Transform as a 7-element csv file saved in given workspace.
    # Assumes data will be type "TransformStamped"

    data_list = [ 
        data.transform.translation.x,
        data.transform.translation.y,
        data.transform.translation.z,
        data.transform.rotation.x,
        data.transform.rotation.y,
        data.transform.rotation.z,
        data.transform.rotation.w
        ]

    outputFilePath = workspace + '/' + filename
    with open(outputFilePath, 'w') as file:
        write = csv.writer(file)
        write.writerow(data_list)

    rospy.loginfo(">> Service Provided: Exported Origin-Camera Transform to %s", outputFilePath)
    rospy.Subscriber.unregister()

    
def listener():
    # Intialize New Node for Subscriber, Wait for Topic to Publish, Subscribe to Topic
    rospy.init_node('tf_origin_to_camera_listener', anonymous=True)
    rospy.wait_for_message('tf_origin_to_camera_transform', TransformStamped, timeout=None)
    rospy.Subscriber("tf_origin_to_camera_transform", TransformStamped, callback)

    #rospy.spin()


if __name__ == '__main__':
    # Required Input Arguments: [absolute file path to workspace, output filename]
    workspace = sys.argv[1]
    filename  = sys.argv[2]

    listener()