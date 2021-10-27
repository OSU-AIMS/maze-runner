

import os



#####################################################
def main():

    print("")
    print("----------------------------------------------------------")
    print("           Maze Runner       (TOP LEVEL)                  ")
    print("----------------------------------------------------------")
    print(" Developed by ACBUYNAK. Spring 2021")
    print("")



    try:


        ## Setup Working Directory
        mzrun_pkg = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        mzrun_ws = mzrun_pkg + '/mzrun_ws'

        try:
            os.makedirs(mzrun_ws, True)
        except OSError:
            if not os.path.isdir(mzrun_ws):
                raise
        os.chmod(mzrun_ws, 0777)



        ## Find Initial Maze Position on Screen
        # Assume maze is within frame of camera
        posej_init = robot_camera.move_group.get_current_joint_values()




        # Testing: Increment Robot Down from overloop position
        if unknown_maze_locn:

            # Move to Start Position
            raw_input('>> Go to Low Crouch Position <enter>')
            start_pose = robot_camera.lookup_pose()
            start_pose.position.z -= 0.7
            robot_camera.goto_Quant_Orient(start_pose)

            # Record First Camera Set
            raw_input('>> Start Finding Maze Path? <enter>')
            timer_wait() #ensure camera has had time to buffer initial frames

            database.capture(img, find_maze=True)   
            latest_data = database.last_recorded()

            last_mazeOrigin       = latest_data['maze_origin']
            last_scale            = latest_data['scale']
            last_tf_camera2world  = np.array(latest_data['tf_camera2world'])
            last_tf_body2camera   = np.array(latest_data['tf_body2camera'])

            print('\n<< Last Maze Information')
            print('last_origin',last_mazeOrigin)
            print('last_scale',last_scale)

            print('last_tf_camera2world (fixed)')
            rot_camera_hardcode  = np.array([[0,-1,0],[-1,0,0],[0,0,-1]])
            translate            = last_tf_camera2world[:-1,-1].tolist()
            last_tf_camera2world = tf_tool.generateTransMatrix(rot_camera_hardcode, translate)
            print(np.around(last_tf_camera2world,2))

            print('last_tf_body2camera')
            print(last_tf_body2camera)

            print('\n<< Maze Body Frame in World Coordinates')
            tf_maze2world = np.matmul(last_tf_camera2world, last_tf_body2camera)
            print(np.around(tf_maze2world,2))




        if path_testing and unknown_maze_locn and False:
            print("\n<< Show Locator Dots")
            latest_data = database.last_recorded
            dots = latest_data['dots']

            # Transform
            dots_as_tf_matrices = prepare_path_tf_ready(dots_list)

            # Find & Apply Rigid Body Transform to Maze 
            # pull from corrected tf in last section
            print('<< Using Maze Origin Frame @ ', tf_maze2world)

            # Rotate Path (as Rigid Body) according to body_frame
            dots_via_fixed_frame = tf_tool.convertPath2FixedFrame(dots_as_tf_matrices, tf_maze2world)

            # Convert Path of Transforms to Robot Poses
            dots_as_path_poses = tf_tool.convertPath2RobotPose(dots_via_fixed_frame)
            orientation = [0.97, 0.242, 0.014, 0.012]

            raw_input('>> Begin Showing Maze Dots <enter>')
            for node in dots_as_path_poses:
                try:
                    # Overwrite Requested Orientation with that of Maze Starting Orientation
                    node[-4:] = orientation

                    # Liftoff Surface for testing purposes
                    node[2] += 0.1

                    print('Moved to Dot @')
                    print(node)

                    robot_pointer.goto_Quant_Orient(node)

                    raw_input('>> Next Dot <enter>')

                except KeyboardInterrupt:
                    break
                    return






        if path_testing and unknown_maze_locn and True:
            print("\n<< Begin Path Testing")

            #  Load Maze Path & Scale
            latest_data = database.last_recorded()
            solved_maze_path = latest_data['maze_soln_filepath']
            last_scale = latest_data['scale']

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
            index =0

            raw_input('>> Begin Solving maze <enter>')
            for node in new_path_poses:
                try:
                    # Overwrite Requested Orientation with that of Maze Starting Orientation
                    node[-4:] = orientation
                    # Liftoff Surface for testing purposes
                    node[2] += 0.04

                    print('Moved to Pose:')
                    print(node)

                    robot_pointer.goto_Quant_Orient(node)

                    if index == 0:
                        raw_input("wait")
                        index +=1
                    #raw_input('>> Next Pose <enter>')

                except KeyboardInterrupt:
                    break
                    return







        # Path Following Demo:
        if fixed_path:

            #maze_closelook_stool = [2.09, -2.4563, 1.037, 0.707, -0.707, 0, 0]
            #robot_camera.goto_Quant_Orient(maze_closelook_stool)

            maze_stool_start = [2.05, -2.47, 0.72, 0.97, 0.242, 0.014, 0.012]
            robot_pointer.goto_Quant_Orient(maze_stool_start)


            # Set Maze to be Solved
            #solved_maze_path = raw_input('Input PreSolved Maze Path: ')
            print("AutoInput Pre-Solved Maze Path:  'lab_demo_soln.csv'")
            solved_maze = '/demo/lab_demo_soln.csv'
            solved_maze_path = mzrun_pkg + solved_maze

            # XY Process Path into Transformation List
            path_as_xyz = prepare_path_transforms_list(solved_maze_path, scaling_factor=0.020)
            path_as_tf_matrices = prepare_path_tf_ready(path_as_xyz)

            # Find & Apply Rigid Body Transform to Maze 
            body_rot = np.matrix('1 0 0; 0 1 0; 0 0 1')   # rot(z,90deg)    ##TODO: get from DREAM3D pipeline  body_rot=retrieve_pose_from_dream3d(_)
            body_transl = np.matrix('1.96846; -2.55415; 0.6200')
            body_frame = tf_tool.generateTransMatrix(body_rot, body_transl)
            print('Maze Origin Frame Calculated to be @ ', body_frame)

            # Rotate Path (as Rigid Body) according to body_frame
            path_via_fixed_frame = tf_tool.convertPath2FixedFrame(path_as_tf_matrices, body_frame)

            # Convert Path of Transforms to Robot Poses
            new_path_poses = tf_tool.convertPath2RobotPose(path_via_fixed_frame)
            starting_orientation = maze_stool_start[-4:]

            raw_input('>> Begin Solving maze <enter>')
            for node in new_path_poses:
                try:
                    # Overwrite Requested Orientation with that of Maze Starting Orientation
                    node[-4:] = starting_orientation

                    print('Moved to Pose:')
                    print(node)

                    robot_pointer.goto_Quant_Orient(node)

                    #raw_input('>> Next Pose <enter>')

                except KeyboardInterrupt:
                    return




        ############################
        # Close Vision Pipeline
        #camera.stopRSpipeline()

        # Move to Known Start Position: All-Zeros
        if motion_testing:
            # raw_input('Go to Maze Overlook Crouch Position <enter>')
            # robot_camera.goto_named_target("maze_overlook_crouch")
            print()


    except rospy.ROSInterruptException:
        # Close Vision Pipeline
        camera.stopRSpipeline()
        return

    except KeyboardInterrupt:
        # Close Vision Pipeline
        camera.stopRSpipeline()
        return


#####################################################
if __name__ == '__main__':
    main()
