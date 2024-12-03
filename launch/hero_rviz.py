from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', '$(find emc_system)/launch/hero.rviz'],
            output='screen'
        )
    ])
