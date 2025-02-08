# RoboCup@Home

## Overview
This project is modified from Carry My Luggage, which is a standard RoboCup task. The goal of the task is for the robot to carry an item specified by the operator, follow the operator around, and return the item to the operator in the end. 
## Table of Contents
- [Prerequisites](#prerequisites)
- [Docker](#Docker)
- [Features](#features)
- [License](#license)

## Prerequisites
- Ubuntu 20.04
- ROS Noetic
- TIAGo Robot / Simulator
- CUDA-compatible GPU (for YOLO)

## Docker
All required packages for this task have been encapsulated in **two Docker images**, which include a fully configured runtime environment. Once downloaded, they can be directly used on the **Tiago robot**.  

**Note:** Due to **differences in MoveIt versions** and **incompatibility of certain message types**, some features may not function correctly in the **Gazebo simulation environment**.  

You can download them here:  
```bash
  docker pull foode258/tiago_yolo:fpmoveit10
  docker pull foode258/yolo_ros:fptest1
```


## Features
- Person Detection and Tracking
  - Real-time detection of people using YOLO V3
  - Continuous tracking through the /text_markers topic
  - Head movement control to track detected persons

- Safe Navigation
  - Maintains configurable safe distance from target person (default: 0.5m)
  - Dynamic navigation goal calculation based on person's position
  - Automatic orientation adjustment to face the person

- Robust Navigation Strategies
  - Automatic retry mechanism for person detection
  - Multiple search rotations when person is not found
  - Detour planning when direct path is blocked
  - Fallback mechanisms for navigation failures

- ROS Integration
  - Compatible with ROS Navigation Stack
  - Uses move_base for path planning and execution 
  - TF2 integration for coordinate transformations
  - Real-time visualization of navigation goals and person poses

- Configuration
  - Adjustable safe distance parameter via launch file
  - Customizable target object detection
  - Configurable retry attempts and intervals


## License

This project is licensed under the MIT License 
