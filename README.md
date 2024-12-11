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

File structure:

📦 Autonomous Search and Retrieval Robot Project
├── 📁 CAD_Files/         # CAD files for robot design (e.g., SolidWorks files, 3D models)
├── 📁 Code/              # Main source code for the robot
│   ├── 📄 robot_control.py       # Code for controlling robot movement and functionality
│   ├── 📄 sensor_integration.py  # Code for processing sensor data
│   ├── 📄 path_planning.py       # Code for navigation and obstacle avoidance
│   └── 📄 utils.py               # Utility functions used across the project
├── 📁 Documentation/     # Documentation and reports
│   ├── 📄 Proposal.pdf           # Project proposal document
│   ├── 📄 Final_Report.pdf       # Final project report
│   ├── 📄 README.md              # Additional documentation
│   └── 📄 References.txt         # List of references used
├── 📁 Simulations/       # Simulation files for testing
│   ├── 📄 simulation_setup.py    # Script to set up and run simulations
│   ├── 📁 Models/                # 3D models used in simulations
│   └── 📁 Results/               # Output from simulations
├── 📁 Media/             # Images, videos, and other media files
│   ├── 📁 Images/               # Screenshots or diagrams of the robot
│   └── 📁 Videos/               # Video demonstrations
├── 📄 LICENSE            # License file for the repository
├── 📄 README.md          # Main README file with an overview of the project
└── 📄 requirements.txt   # List of Python dependencies for the project


## Project Plan & Record of Work

The project plan is available [here](https://sand-weaver-acc.notion.site/Autonomous-Search-and-Retrieval-Robot-Project-11b9b09c1f93800394e0c8895a4e36ce).

The record of work done is avaiable [here](https://docs.google.com/document/d/1Un6J5uqXqME96WFxup4Rg6mEL7XoMLhmGVJzuQLUMJg/edit?usp=sharing).

## Acknowledgements

We extend our gratitude to the University of Manchester and the Robotics Department for their invaluable support and resources, which made this project possible.
