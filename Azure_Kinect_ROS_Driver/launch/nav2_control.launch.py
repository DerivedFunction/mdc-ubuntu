import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    azure_kinect_ros_driver_pkg_dir = get_package_share_directory('azure_kinect_ros_driver')
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')

    # Path to the map file
    map_dir = os.path.join(azure_kinect_ros_driver_pkg_dir, '../../../../../.rtabmap_data')
    map_file = os.path.join(map_dir, 'rtabmap.yaml')  # Nav2 requires a yaml file for the map

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to map yaml file to load'
    )

    # Include Nav2 launch file with our configurations
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': os.path.join(azure_kinect_ros_driver_pkg_dir, 'config', 'nav2_params.yaml')
        }.items()
    )

    # Node to convert Nav2 velocity commands to controller-like outputs
    velocity_to_controller = Node(
        package='laptop',  # You'll need to create this node in your laptop package
        executable='velocity_to_controller',
        name='velocity_to_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Add any additional parameters needed for converting velocities to controller outputs
            'max_linear_speed': 1.0,
            'max_angular_speed': 1.0,
            'joystick_scale': 1.0
        }],
        remappings=[
            ('/cmd_vel', '/nav2/cmd_vel'),  # Subscribe to Nav2's velocity commands
            ('/controller_output', '/laptop/controller_output')  # Publish controller-like outputs
        ]
    )

    # Create and return launch description
    ld = LaunchDescription()
    
    # Add declare arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    
    # Add main nodes
    ld.add_action(nav2_launch)
    ld.add_action(velocity_to_controller)
    
    return ld
