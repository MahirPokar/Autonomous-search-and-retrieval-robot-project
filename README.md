# Autonomous Search and Retrieval Robot Project 

## Introduction

Welcome to the GitHub repository of the Manchester Robotics Team 6! We are a group of four robotics masters students from the University of Manchester: [Euan Baldwin](https://euanbaldwin.github.io), [Mahir Pokar](https://mahirpokar.github.io), [Ao Xiao](https://kkoalayep.github.io) and [Yilin Cao](https://halfmountain4.github.io/). 

This repository serves as our collaborative workspace for developing our robotic systems design project. Our primary objective is to create a robot capable of autonomously picking up objects from its environment.

## Problem Statement

The Leo Rover platform provides an affordable and versatile foundation for robotic applications, yet its base configuration lacks the capabilities needed for object retrieval. To address this, the system will build upon the Leo Rover platform to autonomously navigate a static indoor environment while avoiding obstacles. It must identify specific blocks based on color and place them in matching-colored bins without human intervention. By integrating environmental detection hardware and software along with a manipulator arm, the robot will achieve efficient object retrieval, meeting requirements for modularity, autonomous functionality, reliability, and affordability.

## Hardware

- Chassis - The mechanical base of the robot: **Leo Rover**
- Lidar - Used for mapping and obstacle detection: **SLAMTEC RPLidar A2M12**
- Depth Camera - Provides depth perception for object detection: **Realsense D435**
- Robotic Arm - Responsible for picking up and manipulating objects: **Trossen-PincherX 150**
- Intel NUC - The central processing unit for controlling and coordinating the robot's actions: **SWNUC12WSKi7000**

## Software

Install dependancies:

```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-rplidar-ros
sudo apt install ros-humble-leo-viz
sudo apt install ros-humble-ros-gz
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-librealsense2*
sudo apt install ros-humble-realsense2-*
pip install opencv-python
```

Create a ROS2 workspace:

```bash
mkdir -p ~/leo_ws/src
cd ~/leo_ws/src
```

Clone this repository from github:

```bash
git clone https://github.com/MahirPokar/Autonomous-search-and-retrieval-robot-project.git
```

Source and Build:

```bash
cd ~/leo_ws
source install/setup.bash
colcon build
```

Run simulation:

```bash
ros2 launch *XXXXX*
```

## Acknowledgements

We extend our gratitude to the University of Manchester and the Robotics Department for their invaluable support and resources, which made this project possible.
