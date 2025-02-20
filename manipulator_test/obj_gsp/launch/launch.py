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


    included_launch =IncludeLaunchDescription(
            PythonLaunchDescriptionSource(included_launch_file),
            launch_arguments={'robot_model': 'px150', 'use_sim': 'true'}.items(),
        )

    manipulator_control_node = Node(
        package='obj_gsp',
        executable='manipulator_control',
        name='manipulator_control',
        output='screen'
    )

    pose_publisher_node = Node(
        package='obj_gsp',
        executable='publisher',
        name='publisher',
        output='screen'
    )

    pose_subscriber_node = Node(
        package='obj_gsp',
        executable='pose_subscriber',
        name='pose_subscriber',
        output='screen'
    )

    ld.add_action(included_launch)
    ld.add_action(manipulator_control_node)
    ld.add_action(pose_publisher_node)
    ld.add_action(pose_subscriber_node)
    return ld