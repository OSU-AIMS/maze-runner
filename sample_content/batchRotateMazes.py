import json
import subprocess
import numpy as np

angles = np.arange(0, 180, 1)

filepath_pipelines = '/FILEPATH/TO/PIPELINE/DIRECTORY'
rotationPipeline = filepath_pipelines + '/' + 'rotateImage.json'

filepath_output = '/FILEPATH/TO/OUTPUT/DIRECTORY'
filename_output = 'mazeRot_100x100_'  				#example name included

for angle in angles:
    with open(rotationPipeline, 'r') as jsonFile:
        data = json.load(jsonFile)

    data["1"]["RotationAngle"] = int(angle)

    data["2"]["FileName"] = filepath_output + '/' + filename_output + str(angle) + '.tif'

    with open(rotationPipeline, 'w') as jsonFile:
        json.dump(data, jsonFile)

    subprocess.call(["/opt/dream3d/bin/PipelineRunner", "-p", rotationPipeline])
