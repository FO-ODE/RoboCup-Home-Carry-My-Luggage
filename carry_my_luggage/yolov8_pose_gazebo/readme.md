# YOLOv8-Pose

## Overview
Pose estimation is a task that involves identifying the location of specific points in an image, usually referred to as keypoints. The keypoints can represent various parts of the object such as joints, landmarks, or other distinctive features. The locations of the keypoints are usually represented as a set of 2D `[x, y]` or 3D `[x, y, visible]` coordinates.

We introduce a ROS-based solution for person gesture and posture detection using the TIAGo robot platform, developed for the roboCup@Home competition. The output of a pose estimation model is a set of points that represent the keypoints on an object in the camera, usually along with the confidence scores for each point. Pose estimation is a good choice when you need to identify specific parts of an object in a scene, and their location in relation to each other.

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
- CUDA-compatible GPU
- Ultralytics YOLOv8
- OpenCV & cv_bridge for image processing

## Installation

1. Navigate to the home directory of the ROS workspace where these packages is located:

    ```bash
    cd ~/tiago_ws
    ```

2. Install dependencies:
    ```bash
    sudo apt-get update
    sudo apt install python3-pip
    pip install ultralytics opencv-python rospy cv_bridge numpy
    ```

3. Build the workspace:
    ```bash
    cd ~/tiago_ws
    catkin build
    source devel/setup.bash
    ```

## Features
- Pose Detection & Gesture Recognition
  - Real-time human pose estimation using YOLOv8-Pose.
  - Tracks keypoints such as hands, elbows, shoulders, knees, and head.
  - Publishes pose keypoints for downstream processing.
- Gesture-Based Commands
  - Detects **raised hand gestures** to trigger actions (e.g., stopping robot movement).
  - Recognizes **hand extension gestures** for object interaction.
  - Easily customizable gesture logic.
- Robust Human Tracking
  - Tracks people in **dynamic environments**.
  - Adjusts **robot movement** based on detected keypoints.
  - Works with **multiple people** in the scene.
- ROS Integration
  - Subscribes to **camera image topics**.
  - Publishes detected poses and commands for robot interaction.
  - Supports **TF2 transformations** for seamless integration with other ROS nodes.
- Customization
  - Adjustable keypoint detection confidence thresholds.
  - Configurable **action triggers** for gestures.
  - Compatible with different camera sensors.


## License

This project is licensed under the **MIT License**.
