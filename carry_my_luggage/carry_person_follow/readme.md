# TIAGo Person Following

## Overview
A ROS-based solution for person detection and following using the TIAGo robot platform, developed for the roboCup@Home competition. The system enables TIAGo to detect, track, and safely follow people in dynamic environments.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [License](#license)

## Prerequisites
- ROS Noetic
- Ubuntu 20.04
- TIAGo Robot / Simulator
- Python 3.8+
- CUDA-compatible GPU (for YOLO)

## Installation

1. Create a catkin workspace (skip if you already have one):
    ```bash
    mkdir -p ~/tiago_ws/src
    cd ~/tiago_ws
    catkin init
    ```

2. Clone the repository:
    ```bash
    cd ~/tiago_ws/src
    git clone https://github.com/shilinzhang42/RoboCup-Home.git
    ```

3. Install dependencies:
    ```bash
    sudo apt-get update
    sudo apt install ros-noetic-desktop-full
    ```

4. Install darknet_ros:
    ```bash
    cd ~/tiago_ws/src/final_project
    sudo apt-get update
    git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
    ```

5. Build the workspace:
    ```bash
    cd ~/tiago_ws
    catkin build
    source devel/setup.bash
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
