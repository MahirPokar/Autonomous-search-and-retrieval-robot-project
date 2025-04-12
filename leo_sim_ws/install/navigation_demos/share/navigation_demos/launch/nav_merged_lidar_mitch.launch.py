from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
	
import os	
import launch
import launch_ros.actions


def generate_launch_description():
    ld = LaunchDescription()

    # Parameters, Nodes and Launch files go here

    # Declare package directory
    pkg_nav_demos = get_package_share_directory('navigation_demos')
    # Necessary fixes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behaviour_server',
        'bt_navigator',
    ]

    # LOAD PARAMETERS FROM YAML FILES
    config_bt_nav     = PathJoinSubstitution([pkg_nav_demos, 'config', 'bt_nav.yaml'])
    config_planner    = PathJoinSubstitution([pkg_nav_demos, 'config', 'planner.yaml'])
    config_controller = PathJoinSubstitution([pkg_nav_demos, 'config', 'controller.yaml'])
    params_file = PathJoinSubstitution([pkg_nav_demos, 'config', 'nav_params_for_merged.yaml']) 
    map_file = PathJoinSubstitution([pkg_nav_demos, 'config', 'maze.yaml'])
    explore_lite_launch = PathJoinSubstitution(
        [FindPackageShare('explore_lite'), 'launch', 'explore.launch.py']
    )
    p2s_config = PathJoinSubstitution([pkg_nav_demos, 'config', 'p2s_config.yaml'])
   
   
    # Include Gazebo Simulation
    launch_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('leo_description'), '/launch', '/leo_sim_diff_scan.launch.py']),
    launch_arguments={}.items(),
    )
    
    #launch_rviz = IncludeLaunchDescription(
    #PythonLaunchDescriptionSource([get_package_share_directory('leo_description'), '/launch', '/leo_rviz.launch.py']),
    #launch_arguments={}.items(),
    #)	

    # Include SLAM Toolbox standard launch file
    #launch_slamtoolbox = IncludeLaunchDescription(
    #PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch', '/online_async_launch.py']),
    #launch_arguments={}.items(),
    #)
    
    # Path to your custom SLAM Toolbox parameters YAML
    slam_params_file = os.path.join(
        get_package_share_directory('navigation_demos'),
        'config',
        'slam_params.yaml'
    )

    # Example: pass custom parameters to SLAM Toolbox
    launch_slamtoolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),'launch', 'online_async_launch.py')
        ),
    )
    
    maze_nav=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
        launch_arguments={
        'map':map_file,
        'params_file': params_file}.items(),
    )

    explore_lite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_lite_launch]),
        launch_arguments={}.items(),
    )
    
    scan_malware_launch = LaunchDescription([
        Node(
            package='navigation_demos',
            executable='scan_malware',  # match entry point in setup.py
            name='scan_malware',
            output='screen'
            # parameters=[], # optionally add parameters here if needed
        )
    ])

    pointcloud_to_laserscan = LaunchDescription([
        Node(
            name='pointcloud_to_laserscan',
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            parameters=[p2s_config]
        )
    ])

    
    scan_merger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_laser_scan_merger'), 'launch', 'merge_2_scan.launch.py')
        )
    )


    # Add actions to LaunchDescription
    #ld.add_action(scan_malware_launch)
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    #ld.add_action(explore_lite_launch)
    ld.add_action(launch_gazebo)
    ld.add_action(scan_merger)
    ld.add_action(launch_slamtoolbox)
    ld.add_action(pointcloud_to_laserscan)
    ld.add_action(maze_nav)
    return ld
