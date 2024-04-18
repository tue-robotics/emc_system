#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# Define the launch description
def generate_launch_description():
    ld = LaunchDescription()
    package_name = 'emc_system'
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

    mocap = GroupAction([
        Node(
            package='vrpn_mocap',
            namespace='vrpn_mocap',
            executable='client_node',
            name='vrpn_mocap_client_node',
            output='screen',
            parameters=[{
                'server': server,
                'port': port, 
            }, config]
        ),

        Node(
            package=package_name,
            executable='pose_tf2_broadcaster',
            name='pose_tf2_broadcaster_node',
            output='screen',
            parameters=[{
                'robotname' : 'ROSbot_Coco' 
            }]
        )
    ])
    
    ld.add_action(declare_server)
    ld.add_action(declare_port)
    ld.add_action(mocap)

    return ld