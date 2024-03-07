import os
import re
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    
    # Package Directories
    pkg_go2_ign_description = get_package_share_directory('go2_ign_description')
    
    # Parse robot description from URDF
    robot_description_file = os.path.join(pkg_go2_ign_description, 'models', 'urdf', 'go2_description.urdf')
    with open(robot_description_file, 'r') as file:
        robot_description = file.read()

    robot_description = {"robot_description": robot_description}
    
    # send fake joint values
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_gui': True}]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=["-d", os.path.join(pkg_go2_ign_description, 'rviz', 'joint_state.rviz')]
    )
    
    return LaunchDescription(
        [
            joint_state_publisher_gui,
            robot_state_publisher,
            rviz2,
        ]
    )
