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
import json

# Custom Scripts
from process_path import prepare_path_transforms_list, prepare_path_tf_ready

## SUPPORT FUNCTIONS ##
def rosmsg_geoPose(pose):
    ## Recieves dictionary and list formats and returns a tf2.geom.pose message

    # Get Current Orientation in Quanternion Format
    # http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html
    #q_poseCurrent = self.move_group.get_current_pose().pose.orientation
    #print(q_poseCurrent)

    # Using Quaternion's for Angle
    # Conversion from Euler(rotx,roty,rotz) to Quaternion(x,y,z,w)
    # Euler Units: RADIANS
    # http://docs.ros.org/en/melodic/api/tf/html/python/transformations.html
    # http://wiki.ros.org/tf2/Tutorials/Quaternions
    # http://docs.ros.org/en/api/geometry_msgs/html/msg/Quaternion.html

    if isinstance(pose,list):
        # Assume already in Quanternion angles in format xyzw
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]
        pose_goal.orientation.x = pose[3]
        pose_goal.orientation.y = pose[4]
        pose_goal.orientation.z = pose[5]
        pose_goal.orientation.w = pose[6]
    else:
        print("---> Incorrect Input Format")

    return pose_goal


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
    message.header.frame_id = 'origin'
    message.poses = inputArray
    
    # Publish node
    while not rospy.is_shutdown():
        #rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()



def last_recorded(mzrun_ws):
    # Build Temporary Dictionary to Return latest Data

    filename = str(mzrun_ws) + '/camera_poses.json'
    print(filename)

    with open(filename) as f:
        all_data = json.load(f)

    latest_data = {
        'counter': all_data['counter'][-1],
        'images': all_data['images'][-1],
        'poses' : all_data['poses'][-1],
        'maze_centroid' : all_data['maze_centroid'][-1],
        'maze_rotationMatrix' : all_data['maze_rotationMatrix'][-1],
        'scale' : all_data['scale'][-1],
        'maze_origin' : all_data['maze_origin'][-1],
        'maze_soln_filepath' : all_data['maze_soln_filepath'][-1],
        'tf_camera2world' : all_data['tf_camera2world'][-1], 
        'tf_body2camera' : all_data['tf_body2camera'][-1],
        'dots' : all_data['dots'][-1],
    }

    return latest_data



## MAIN CODE ##
def main():

    #Copied from maze_runner.py
    import os
    import numpy as np
    from transformations import transformations
    tf_tool = transformations()

    mzrun_pkg = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    mzrun_ws = mzrun_pkg + '/mzrun_ws'


    if True:

         # Open Data
        from maze_runner import DataSaver
        latest_data = last_recorded(mzrun_ws)

        print('\n<< Load Database w/ Last Information Saved')
        solved_maze_path      = latest_data['maze_soln_filepath']
        last_scale            = latest_data['scale']
        last_mazeOrigin       = latest_data['maze_origin']
        last_scale            = latest_data['scale']
        last_tf_camera2world  = np.array(latest_data['tf_camera2world'])
        last_tf_body2camera   = np.array(latest_data['tf_body2camera'])


        # Manually adjusting Camera Transform
        rot_camera_hardcode  = np.array([[0,-1,0],[-1,0,0],[0,0,-1]])
        translate            = last_tf_camera2world[:-1,-1].tolist()
        last_tf_camera2world = tf_tool.generateTransMatrix(rot_camera_hardcode, translate)
        
        print('\n<< Maze Body Frame in World Coordinates')
        tf_maze2world = np.matmul(last_tf_camera2world, last_tf_body2camera)
        print(np.around(tf_maze2world,2))


        # XY Process Path into Transformation List
        path_as_xyz = prepare_path_transforms_list(solved_maze_path, scaling_factor=last_scale)
        path_as_tf_matrices = prepare_path_tf_ready(path_as_xyz)

        # Find & Apply Rigid Body Transform to Maze 
        # pull from corrected tf in last section
        print('<< Using Maze Origin Frame @ ', tf_maze2world)

        # Rotate Path (as Rigid Body) according to body_frame
        path_via_fixed_frame = tf_tool.convertPath2FixedFrame(path_as_tf_matrices, tf_maze2world)

        # Convert Path of Transforms to Robot Poses
        new_path_poses = tf_tool.convertPath2RobotPose(path_via_fixed_frame)
        orientation = [0.97, 0.242, 0.014, 0.012]


        pose_geom = []
        for node in new_path_poses:
            node[-4:] = orientation
            
            # Liftoff Surface for testing purposes
            node[2] += 0.04
            pose_geom.append(rosmsg_geoPose(node))


    if False:
        print("AutoInput Pre-Solved Maze Path:  'lab_demo_soln.csv'")
        # Process Path into Flat Vector Plane
        path_as_xyz = prepare_path_transforms_list(solved_maze_path,scaling_factor=0.02)

        # Convert Cartesian Points to Transformation List
        path_as_tf_matrices = prepare_path_tf_ready(path_as_xyz)

        # Create Maze Origin POSE
        import numpy as np
        from transformations import transformations
        tf = transformations()

        #body_rot = np.matrix('0 -1 0; 1 0 0; 0 0 1')   # rot(z,90deg)
        body_rot = np.matrix('1 0 0; 0 1 0; 0 0 1')   # rot(z,90deg)    ##TODO: get from DREAM3D pipeline  body_rot=retrieve_pose_from_dream3d(_)
        body_transl = np.matrix('1.96846; -2.55415; 0.6500')
        body_frame = tf.generateTransMatrix(body_rot, body_transl)
        #print(body_frame)

        path_via_fixed_frame = tf.convertPath2FixedFrame(path_as_tf_matrices, body_frame)

        # Convert Path of Transforms to Robot Poses
        new_path_poses = tf.convertPath2RobotPose(path_via_fixed_frame)
        print(new_path_poses)



        # Generate PoseArray for ROS Node Publisher
        pose_geom = []
        for i in new_path_poses:
            i[-4:] = starting_orientation
            pose_geom.append(rosmsg_geoPose(i))



    # Try launching ros node
    try:
        node_cameraPoseArray(pose_geom)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
