import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Package Directories
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_go2_ign_description = get_package_share_directory('go2_ign_description')

    # Parse robot description from xacro
    robot_description_file =  os.path.join(pkg_go2_ign_description, 'models', 'xacro', 'robot.xacro')
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_desc = robot_description_config.toxml()

    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': True, 
            'robot_description': robot_desc
        }],
    )
    
    # TODO(zhong): joint still not working. What is the role of tf_footprint_base ?
    # Static transform publisher 
    tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen',
    )

    # Ignition gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
            launch_arguments={'ign_args': '-r empty.sdf'}.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', 'go2',
            '-topic', 'robot_description',
        ],
        output='screen',
    )

    return LaunchDescription(
        [   
            tf_footprint_base,
            robot_state_publisher,
            gazebo,
            spawn,
        ]
    )