from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='depth', default_value='depth',
            description='Namespace for sample topics'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '-0.1',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'd435i_camera_link', '--child-frame-id', 'cloud'
            ]
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', [LaunchConfiguration(variable_name='depth'), '/points']),
                        ('scan', [LaunchConfiguration(variable_name='depth'), '/scan'])],
            parameters=[{
                'target_frame': 'cloud',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 0.2,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.15,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
