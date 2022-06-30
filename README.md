# marker_detection

## Overview
This package will detect one or more ArUco Markers and broadcast each tfs(Transforms).

## Dependencies
- Ubuntu 20.04 LTS (Focal Fossa)
- Robotic Operating System([ROS](http://wiki.ros.org/ROS/Installation) noetic

To activate it, you will need
- pyrealsense2
- OpenCV
- open3d

## Notice
The code in this package is tested on **python3** environment

## Installation
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/CoMoBot-Perception/marker-detection.git
$ cd ~/catkin_ws/
$ catkin_make
```

## How to start?
```
$ roslaunch marker_detection marker_detection.launch
```
## Reference
- **[OpenCV: Detection of ArUco Markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)**