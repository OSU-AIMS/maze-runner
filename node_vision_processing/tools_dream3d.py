



def runD3D_mazeLocators(input_image_path, result_feature_path, result_maze_path, locator_scalar_tolerance=1000):
    """
    Processor caller for preconfigured Dream3D pipeline.
    Requires: os, subprocess, json, rospy, rospkg

    Input:  RGB image 
    Output: Centroid and Size of red, green, blue color clusters
    """

    import os
    import subprocess
    import json
    import rospy
    import rospkg

    rospack       = rospkg.RosPack()
    workspace_dir = rospack.get_path('maze_runner')
    log_dir       = os.path.join(workspace_dir, 'mzrun_ws') #os.environ.get('ROS_LOG_DIR')

    pipeline = os.path.join(workspace_dir, 'dream3d_pipelines/filter_irs_image.json')

    # Setup & Run Dream3D / Simple Pipeline
    with open(pipeline, 'r') as jsonFile:
        data = json.load(jsonFile)

    data["00"]["FileName"] = input_image_path

    # Feature Segmenter Settings
    data["14"]["ScalarTolerance"] = locator_scalar_tolerance
    data["15"]["ScalarTolerance"] = locator_scalar_tolerance
    data["16"]["ScalarTolerance"] = locator_scalar_tolerance

    # Desired Outputs
    data["23"]["FeatureDataFile"] = result_feature_path[0]  # RED
    data["24"]["FeatureDataFile"] = result_feature_path[1]  # GREEN
    data["25"]["FeatureDataFile"] = result_feature_path[2]  # BLUE
    data["31"]["FileName"]        = result_maze_path        # MAZE

    # Debugging Tools
    data["26"]["OutputFilePath"] = os.path.join(log_dir, 'vision_debug/feature_redDot_array.csv')
    data["28"]["OutputFilePath"] = os.path.join(log_dir, 'vision_debug/filter_irs_image.dream3d')
    data["34"]["FileName"]       = os.path.join(log_dir, 'vision_debug/single_color_mask.tiff')

    # Write out updated Json
    with open(pipeline, 'w') as jsonFile:
        json.dump(data, jsonFile, indent=4, sort_keys=True)

    # Workaround to supress output.
    d3d_output_path = os.path.join(log_dir, 'log_D3D_mazeLocators.txt')
    d3d_output = open(d3d_output_path, 'w')
    subprocess.call(["/opt/dream3d/bin/PipelineRunner", "-p", pipeline], stdout=d3d_output, stderr=d3d_output)

    rospy.logdebug("Dream3D Pipeline Runner Complete.")
    return 1



def runD3D_mazePath():
    print("placeholder")