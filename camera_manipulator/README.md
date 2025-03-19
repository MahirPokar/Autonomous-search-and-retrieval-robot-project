# **Interbotix X-Series Arms with ROS 2 Humble on Ubuntu Linux 22.04**

## **Installation for Intel/AMD Processors**
If your computer uses an Intel or AMD processor (such as NUCs, most laptops, and desktop computers), follow the commands below to download and run the installation script:

```bash
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d humble
```

## **Running the Package on a Physical Robot**
In different terminals, run the following commands to start the physical robot:

```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px150
```

In another terminal, run:

```bash
ros2 run obj_gsp detect_object | ros2 run obj_gsp manipulator_control
# OR
ros2 launch obj_gsp leolaunch1.py
```

## **Updating the Model Data**
If you have the latest trained data, replace the model file located at:

```
/obj_gsp/models/best_copy.pt
```

> 📂 **Note:** There are several old versions in the folder; you can change it if needed.

