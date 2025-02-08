# RoboCup@Home

## Overview
This project is modified from Carry My Luggage, which is a standard RoboCup task. The goal of the task is for the robot to carry an item specified by the operator, follow the operator around, and return the item to the operator in the end. 
## Table of Contents
- [Prerequisites](#prerequisites)
- [Docker](#Docker)
- [Features](#features)
- [Usage](#Usage)
## Prerequisites
- Ubuntu 20.04
- ROS Noetic
- TIAGo Robot / Simulator
- CUDA-compatible GPU (for YOLO)

## Docker
All required packages for this task have been encapsulated in **two Docker images**, which include a fully configured runtime environment. Once downloaded, they can be directly used on the **Tiago robot**. And it is recommended to boot the image with **rocker** in order to enable Nvidia CUDA support.

**Note:** Due to **differences in MoveIt versions** and **incompatibility of certain message types**, some features may not function correctly in the **Gazebo simulation environment**.  

You can download them here:  
```bash
  docker pull foode258/tiago_yolo:fpmoveit10
  docker pull foode258/yolo_ros:fptest1
```




## Usage
The navigation points within the program are based on the map in the lab, please adjust accordingly.

Please make sure that dockers can communicate with each other properly.




In the **yolo_ros** image:
```bash
roslaunch object_detection object_detection.launch
```
In the **tiago_yolo** image：
```bash
rosrun carry_task_manager task_manager.py
```





## Features
- Person Detection and Tracking
  - Real-time detection of people using YOLO V3
  - Continuous tracking through the /text_markers topic
  - Head movement control to track detected persons


- ROS Integration
  - Compatible with ROS Navigation Stack
  - Uses move_base for path planning and execution 
  - TF2 integration for coordinate transformations
  - Real-time visualization of navigation goals and person poses


