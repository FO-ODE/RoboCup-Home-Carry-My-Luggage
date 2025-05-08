# RoboCup@Home

## Overview
This project is modified from Carry My Luggage, which is a standard RoboCup task. The goal of the task is for the robot to carry an item specified by the operator, follow the operator around, and return the item to the operator in the end. 


## Table of Contents
- [Prerequisites](#prerequisites)
- [Docker](#docker)
- [Ultralytics](#ultralytics)
- [Usage](#usage)
- [Demo Video](#demo-video)
- [External Links](#external-links)

## Prerequisites
- Ubuntu 20.04
- ROS Noetic
- TIAGo Robot / Simulator
- CUDA-compatible GPU (for YOLO)
- Ultralytics

## Docker
Most required packages for this task have been encapsulated in two Docker images, which include a fully configured runtime environment. Once downloaded, they can be directly used with the TIAGo robot. And it is recommended to boot the image with **rocker** in order to enable Nvidia CUDA support.

**Note:** Due to differences in MoveIt versions and incompatibility of certain message types, some features may not function correctly in the Gazebo simulation environment.  

You can download them here:  
```bash
  docker pull foode258/tiago_yolo:fpmoveit10
  docker pull foode258/yolo_ros:fptest1
```

## Ultralytics
We have not integrated Ultralytics in the docker image, which is used for character pose and gesture recognition. With ROS Noetic and tiago_tutorials installed, follow these steps to install Ultralytics:
```bash
  cd ~/tiago_ws
```

Install the dependencies: 
```bash
sudo apt-get update
sudo apt install python3-pip
pip install ultralytics opencv-python numpy
sudo apt-get install ros-noetic-rospy ros-noetic-cv-bridge
```

Build the ROS workspace: 
```bash
  catkin build
  source devel/setup.bash
```





## Usage
The navigation points within the program are based on the map in the lab, please adjust accordingly.

Please make sure that dockers can communicate with each other properly.


In the local Terminal:
```bash
rosrun yolov8_pose_gazebo yolov8_pose_gazebo.py
```

In the Docker container based on the **yolo_ros** image:
```bash
roslaunch object_detection object_detection.launch
```
In the Docker container based on the **tiago_yolo** image:
```bash
roslaunch carry_navi localization.launch
rosrun carry_task_manager task_manager.py
```
Then the TIAGo robot will perform the Carry My Luggage task. 

Please refer to the instructions within the task manager if you need detailed steps.

## Demo Video
[![Click to play](https://img.youtube.com/vi/OzqW0VTr4R4/0.jpg)](https://youtu.be/OzqW0VTr4R4)

## External Links
https://wiki.ros.org/Robots/TIAGo/Tutorials/Installation

https://wiki.ros.org/darknet_ros

https://docs.ultralytics.com/tasks/pose




