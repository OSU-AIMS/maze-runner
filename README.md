# Maze Runner :robot: :runner:

<a href="https://docs.ros.org/en/foxy/index.html"><img src="https://img.shields.io/badge/ROS-Foxy-blue"/></a>
<a href="https://www.ros.org/reps/rep-2004.html"><img src="https://img.shields.io/badge/REP_2004-Quality_4-red"/></a>

Sample package for using a 6-axis robotic arm to navigate a maze. Intended to support the development of robotic applications at the Artifically Intelligent Manufacturing Systems (AIMS) Lab at The Ohio State University.

_Note: This repo is designed for ROS 2 (Foxy+). Previous verions of code may support ROS 1 enviornments._
_The Motoman MH5 robot was used as the primary hardware environment._


## Installation

1. Setup Workspaces
```
mkdir -p <worskpace_tesseract>/src
mkdir -p <workspace_maze_runner>/src
```

2. Pull this repo local
```
cd <workspace_maze_runner>/src
git clone https://github.com/osu-aims/maze-runner
```

3. Install Tesseract Dependencies (unstable as of July 2022)
Build Tesseract in a separate workspace such that it only needs built once.
```
cd <worskpace_tesseract>/src
vcs import < <workspace_maze_runner>/src/maze-runner/dependencies-tesseract.rosinstall

cd ..
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DTESSERACT_BUILD_FCL=OFF
exit
```

4. Build Maze-Runner Workspace
New terminal. Pull dependencies.
```
cd <workspace_maze_runner>/src
vcs import < ./maze-runner/dependencies.rosinstall
cd ..
```

Overlay previously built Tesseract workspace
```
source <worskpace_tesseract>/install/setup.bash
colcon build --symlink-install
```

5. Setup for Usage in New Terminal
```
source <workspace_maze_runner>/install/setup.bash
```


## Usage
`todo`
