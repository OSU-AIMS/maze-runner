# Maze Runner :robot: :runner:
Sample package for using a 6-axis robotic arm to navigate a maze. Intended to support the development of robotic applications at the Artifically Intelligent Manufacturing Systems (AIMS) Lab at The Ohio State University.

_Note: This repository was designed for ROS Melodic. It has not been tested on other distributions._
_Specifically designed for the Motoman MH5 robot as supported by the ROS-Industrial program._

## Installation
The following section explains how to setup the package to work.

### Prerequisites
  - **ROS Melodic:** For obtaining and configuring ROS follow the installation instructions for [full-desktop Melodic installation](http://wiki.ros.org/melodic/Installation/Ubuntu).
  - **Catkin workspace:** Create a clean [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/index.html) workspace.
  - **MoveIt 1:** For installation instructions see [MoveIt's webpage](https://moveit.ros.org/install/).

### Required Repositories
  Clone the below repositories into your catkin-tools workspace:
  - [maze_runner](https://github.com/OSU-AIMS/maze_runner)
  - [ros-industrial/Motoman](https://github.com/ros-industrial/motoman)


## Usage

```
roslaunch motoman_mh5_moveit_config demo.roslaunch
rosrun maze_runner follow_solved_maze.py
```
