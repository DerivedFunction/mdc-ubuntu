import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Paths
    pkg_share = get_package_share_directory('azure_kinect_ros_driver')
    
    # Don't worry about this: part of nav2
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    # Params file
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([

        # --- DepthImage → LaserScan ---
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            remappings=[
                ('depth', '/depth_to_rgb/image_raw'),
                ('depth_camera_info', '/depth_to_rgb/camera_info'),
                ('scan', '/scan')
            ],
            parameters=[{
                'range_max': 4.0,
                'scan_height': 1,
                'output_frame': 'camera_base'  # or base_link depending on your TF
            }]
        ),

        # --- Nav2 bringup ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false'
            }.items(),
        ),
    ])
