#!/usr/bin/env python3

import os.path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    package_dir = get_package_share_directory('emc_system')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen' 
    )
    
    ld.add_action(rviz2)

    return ld 
