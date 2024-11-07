cd ~/ros2_ws
colcon build #--symlink-install
source ./install/setup.bash
sudo chmod 777 /dev/ttyUSB0
cd src/rplidar_ros/
source scripts/create_udev_rules.sh
ros2 launch rplidar_ros view_rplidar_a2m12_launch.py
