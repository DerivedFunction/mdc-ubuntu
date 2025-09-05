import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='./map.yaml',
            description='Full path to map yaml file'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
