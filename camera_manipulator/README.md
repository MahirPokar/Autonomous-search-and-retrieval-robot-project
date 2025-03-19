#The Interbotix X-Series Arms are compatible with the ROS 2 Humble on Ubuntu Linux 22.04


If your computer uses an Intel or AMD based processor (which is the case for NUCs, most laptops and desktop computers), follow the commands below to download and run the installation script. 

  sudo apt install curl
  
  curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
  
  chmod +x xsarm_amd64_install.sh
  
  ./xsarm_amd64_install.sh -d humble


To run this package on a physical robot, run the command below in different terminals
  ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px150
  ros2 run obj_gsp detect_object | ros2 run obj_gsp manipulator_control   OR   ros2 launch obj_gsp leolaunch1.py

                                                       
If you have the latest trained data, change the/obj_gsp/models/best_copy.pt


There are several old versions in the folder,change it if you want.
