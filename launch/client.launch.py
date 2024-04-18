#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# Define the launch description
def generate_launch_description():
    ld = LaunchDescription()
    vrpn_mocap = get_package_share_directory('vrpn_mocap')
    server = LaunchConfiguration('server')
    port = LaunchConfiguration('port')
    config = os.path.join(vrpn_mocap, 'config', 'client.yaml')

    declare_server = DeclareLaunchArgument(
        'server',
        default_value='192.168.5.101',
    )

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='3883',
    )

    mocap = Node(
        package='vrpn_mocap',
        namespace='vrpn_mocap',
        executable='client_node',
        name='vrpn_mocap_client_node',
        output='screen',
        parameters=[{
            'server': server,
            'port': port, 
        }, config]
    )

    ld.add_action(declare_server)
    ld.add_action(declare_port)
    ld.add_action(mocap)

    return ld