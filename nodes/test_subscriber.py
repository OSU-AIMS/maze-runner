
import subprocess
import tf_origin_camera

# Required Input Arguments: [absolute file path to workspace, output filename]
workspace = '/home/aims-zaphod/AA_DEVL/ws_mazerunner_multibot/src/maze_runner/mzrun_ws'
filename = 'cool_file.csv'


subprocess.call(['/home/aims-zaphod/AA_DEVL/ws_mazerunner_multibot/src/maze_runner/services/tf_origin_camera.py', workspace, filename] )
print('Done')