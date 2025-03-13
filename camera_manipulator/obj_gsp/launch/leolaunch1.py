from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    interbotix_pkg_dir = get_package_share_directory('interbotix_xsarm_control')
    included_launch_file = os.path.join(interbotix_pkg_dir, 'launch', 'xsarm_control.launch.py')

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_file),
        launch_arguments={'robot_model': 'px150', 'use_sim': 'true'}.items(),
    )

    manipulator_control_node = Node(
        package='obj_gsp',
        executable='manipulator_control',
        name='manipulator_control',
        output='screen'
    )

    detect_object_node = Node(  # 添加目标检测节点
        package='obj_gsp',
        executable='detect_object',
        name='detect_object',
        output='screen'
    )

    ld.add_action(included_launch)
    ld.add_action(manipulator_control_node)
    ld.add_action(detect_object_node)  # 确保目标检测也在 launch 时启动

    return ld

