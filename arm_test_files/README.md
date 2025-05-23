# Interbotix X-Series Arm Python API Demos

[![docs](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/python_demos.html)

## Overview

This directory showcases various ways of using the [Interbotix Python Arm Module](https://github.com/Interbotix/interbotix_ros_toolboxes/tree/galactic/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/arm.py) (click the link to see the fully documented code; also check out the [interbotix_xs_modules.xs_robot library](https://github.com/Interbotix/interbotix_ros_toolboxes/tree/galactic/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot) to get a better understanding on how this and other modules can be used). Simply put, this API was created so that users with little to no ROS experience would still have the ability to control any Interbotix Arm supported by the *interbotix_xs_sdk*. Specifically, the API also allows a user to make an arm go to desired end-effector poses or follow Cartesian trajectories. This last feature was made possible by the [Modern Robotics: Mechanics, Planning, and Control Code Library](https://github.com/NxRLab/ModernRobotics) created at Northwestern University.

For the API to work, the arm joints must be set to 'position' control and the gripper set to 'PWM' control (conveniently, these are the default configs in the *interbotix_xsarm_control* package). Furthermore, the API assumes that all the arm-joint motors' [Drive Mode](http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#drive-mode) registers are set to [Time-Based-Profile](http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#profile-velocity112) (this is also the default configuration). In a nutshell, this setting makes it very easy for you as the user to customize the duration and smoothness of an arm's motion from one pose to the next.


In one terminal run the following command:
```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px150
```

In another terminal run the desired python file: 
```bash
python3 simple_test.py
```
Make sure the demos folder is within /interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control 

To do: 
Make a package that only imports the necessary packages from the Interbotix library. 

