#!/usr/bin/env python
#
# Support file for node: vision_processing
# 

def runD3D_mazeLocators(fpath_image, dot_names, locator_scalar_tolerance=1000, local_debug="False"):
    """
    Processor caller for preconfigured Dream3D pipeline.
    Requires: os, subprocess, json, rospy, rospkg

    :param fpath_image: Absolute filepath to maze image
    :param dot_names:   List of Strings, 3-element. Order MUST be red, green, blue. But exact names may be changed.
    return: Centroid and Size of red, green, blue color clusters
    """

    import os
    import subprocess
    import json
    import rospy
    import rospkg

    # Directories and Filepaths
    rospack     = rospkg.RosPack()
    dir_pkg     = rospack.get_path('maze_runner')
    dir_wksp    = os.path.join(dir_pkg, 'mzrun_ws')
    dir_log     = dir_wksp      #os.environ.get('ROS_LOG_DIR')
    pipeline    = os.path.join(dir_pkg, 'dream3d_pipelines/filter_irs_image.json')
    
    fpath_redDot     = os.path.join(dir_wksp, 'feature_' + dot_names[0] + '.csv')
    fpath_greenDot   = os.path.join(dir_wksp, 'feature_' + dot_names[1] + '.csv')
    fpath_blueDot    = os.path.join(dir_wksp, 'feature_' + dot_names[2] + '.csv')
    fpath_maze       = os.path.join(dir_wksp, 'mask_MazeOnly.tiff')


    # Setup & Run Dream3D / Simple Pipeline
    with open(pipeline, 'r') as jsonFile:
        data = json.load(jsonFile)
        data["00"]["FileName"] = fpath_image

        # Feature Segmenter Settings
        data["18"]["ScalarTolerance"] = locator_scalar_tolerance
        data["19"]["ScalarTolerance"] = locator_scalar_tolerance
        data["20"]["ScalarTolerance"] = locator_scalar_tolerance

        # Desired Outputs
        data["27"]["FeatureDataFile"] = fpath_redDot    # RED
        data["28"]["FeatureDataFile"] = fpath_greenDot  # GREEN
        data["29"]["FeatureDataFile"] = fpath_blueDot   # BLUE
        data["33"]["FileName"]        = fpath_maze      # MAZE

        # Debugging Tools
        data["34"]["Filter_Enabled"] = local_debug
        data["34"]["OutputFile"]     = os.path.join(dir_log, 'pipeline_filter_irs_image.dream3d')

    # Write out updated Json
    with open(pipeline, 'w') as jsonFile:
        json.dump(data, jsonFile, indent=4, sort_keys=True)

    # Workaround to supress output.
    d3d_output_path = os.path.join(dir_log, 'log_D3D_mazeLocators.txt')
    d3d_output = open(d3d_output_path, 'w')
    subprocess.call(["/opt/dream3d/bin/PipelineRunner", "-p", pipeline], stdout=d3d_output, stderr=d3d_output)

    rospy.logdebug("runD3D_mazeLocators: Runner Complete.")

    return (fpath_maze, [fpath_redDot, fpath_greenDot, fpath_blueDot])



def runD3D_maurerFilter(input_maze_image):
    """
    Processor caller for preconfigured Dream3D pipeline.
    Requires: os, subprocess, json, rospy, rospkg

    :param input_maze_image:    Absolute file path to the cropped maze
    :return output_maurer_path: Absolute file path to the Maurer Filtered maze image
    """
    import os
    import subprocess
    import json
    import rospy
    import rospkg

    rospack = rospkg.RosPack()
    dir_pkg = rospack.get_path('maze_runner')
    dir_log = os.path.join(dir_pkg, 'mzrun_ws') #os.environ.get('ROS_LOG_DIR')

    pipeline = os.path.join(dir_pkg, 'dream3d_pipelines/filter_maurer_path.json')
    output_maurer_path = os.path.join(dir_log, 'maze_maurer_path.tiff')

    # Setup Dream3D / Simple Pipeline
    with open(pipeline, 'r') as jsonFile:
        data = json.load(jsonFile)
        data["0"]["FileName"] = input_maze_image
        data["7"]["FileName"] = output_maurer_path

    with open(pipeline, 'w') as jsonFile:
        json.dump(data, jsonFile, indent=4)

    # Run Dream3D Pipeline
    d3d_output_path = os.path.join(dir_log, 'log_D3D_maurerPath.txt')
    d3d_output = open(d3d_output_path, 'w')
    subprocess.call(["/opt/dream3d/bin/PipelineRunner", "-p", pipeline], stdout=d3d_output, stderr=d3d_output)

    return output_maurer_path