import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
import xacro


def generate_launch_description():
    ld = LaunchDescription()
    

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'leo_description'
    file_subpath = 'urdf/leo.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # joint state publisher (GUI) node
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    
    #add the ld.add action for bridge. 	
    
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
                    '/clock'                           + '@rosgraph_msgs/msg/Clock'   + '[' + 'ignition.msgs.Clock',
                    #'/model/leo/cmd_vel'  + '@geometry_msgs/msg/Twist'   + '@' + 'ignition.msgs.Twist',
                    '/model/leo/odometry' + '@nav_msgs/msg/Odometry'     + '[' + 'ignition.msgs.Odometry',
                    '/model/leo/scan'     + '@sensor_msgs/msg/LaserScan' + '[' + 'ignition.msgs.LaserScan',
                    '/model/leo/tf'       + '@tf2_msgs/msg/TFMessage'    + '[' + 'ignition.msgs.Pose_V',
                    '/model/leo/imu'      + '@sensor_msgs/msg/Imu'       + '[' + 'ignition.msgs.IMU',
                    '/world/empty/model/leo/joint_state' + '@sensor_msgs/msg/JointState' + '[' + 'ignition.msgs.Model',
                    ],
        parameters= [{'qos_overrides./gz_example_robot.subscriber.reliability': 'reliable'}],
        remappings= [
                    #('/model/leo/cmd_vel',  '/cmd_vel'),
                    ('/model/leo/odometry', '/odom'   ),
                    ('/model/leo/scan',     '/scan'   ),
                    ('/model/leo/tf',       '/tf'     ),
                    ('/model/leo/imu',      '/imu_raw'),
                    ('/world/empty/model/leo/joint_state', 'joint_states')
                    ],
        output='screen'
    )

    # Rviz node
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory(pkg_name), 'rviz', 'leo_view.rviz')]
    )


    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=False))
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_joint_state_publisher)
    ld.add_action(node_rviz)
    return ld
