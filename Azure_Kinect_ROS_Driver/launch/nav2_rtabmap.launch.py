import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

# --- Interactive DB selection (preserve existing behavior) ---
dbname = input("Enter map name: ").strip()
if not dbname:
    dbname = "rtabmap.db"
if not dbname.endswith(".db"):
    dbname += ".db"

# --- Paths ---
azure_kinect_ros_driver_pkg_dir = get_package_share_directory('azure_kinect_ros_driver')
rtabmap_ros_pkg_dir = get_package_share_directory('rtabmap_ros')
nav2_ros_pkg_dir = get_package_share_directory('nav2_bringup')

urdf_file_path = os.path.join(azure_kinect_ros_driver_pkg_dir, 'urdf', 'azure_kinect.urdf.xacro')
rviz_config_file = os.path.join(os.getcwd(), "config", "nav2_slam.rviz")


save_location = os.path.join(os.getcwd(), "rtabmap_data")
default_db_path = os.path.join(save_location, dbname)

# Default nav2 params YAML inside this package's config folder
default_nav2_params = os.path.join(os.getcwd(), "config", "nav2_params.yaml")

# nodes that lifecycle manager will manage
lifecycle_nodes = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower'
    # 'velocity_smoother'
]

# --- Base parameters for RTAB-Map ---
rtabmap_parameters = {
    "frame_id": "camera_base",
    "subscribe_depth": True,
    "subscribe_rgb": True,
    "use_sim_time": LaunchConfiguration("use_sim_time"),
    "approx_sync": True,
    "approx_sync_max_interval": 0.04,
    # QoS settings: 2 = SensorDataQoS
    "qos_image": 2,
    "qos_imu": 2,
    # ODOMETRY PARAMETERS
    "Reg/Strategy": "0",  # Visual Odometry
    "Odom/Strategy": "0",  # Frame-to-Frame tracking
    "Vis/MinInliers": "15",
    "Odom/ResetCountdown": "10",
    "Vis/FeatureType": "8",  # ORB
    "OdomF2M/MaxSize": "1000",
    # LOOP CLOSURE & MAPPING
    "Grid/FromDepth": "true",
    "Reg/Force3DoF": "true",
    "Grid/RangeMax": "5.0",
    "database_path": default_db_path,
    # Obstacle detection and raytracing
    "Grid/MinGroundHeight": "-0.25",
    "Grid/MaxGroundHeight": "0.05",
    "Grid/MaxObstacleHeight": "0.5",
    "Grid/RayTracing": "true",
    "Grid/MapFrameProjection": "true",
    # Reuse local map
    "Mem/IncrementalMemory": "false",
}

# --- Remapping Kinect topics to RTAB-Map ---
rtabmap_remapping = [
    ('rgb/image', '/rgb/image_raw'),
    ('rgb/camera_info', '/rgb/camera_info'),
    ('depth/image', '/depth_to_rgb/image_raw')
]

# --- Configured params substitution for Nav2 nodes ---
configured_params = [LaunchConfiguration('params_file')]

def generate_launch_description():
    ld = LaunchDescription()

    # Common launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false',
                                        description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument('db_path', default_value=default_db_path,
                                        description='Path to store or load the RTAB-Map database file'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='true',
                                        description='Automatically startup the nav2 stack'))
    ld.add_action(DeclareLaunchArgument('container_name', default_value='nav2_container',
                                        description='the name of container that nodes will load in if use composition'))
    ld.add_action(DeclareLaunchArgument('use_respawn', default_value='False',
                                        description='Whether to respawn if a node crashes. Applied when composition is disabled.'))
    ld.add_action(DeclareLaunchArgument('log_level', default_value='info',
                                        description='log level'))
    # Nav2 params file argument (points to azure_kinect/config/nav2_params.yaml by default)
    ld.add_action(DeclareLaunchArgument('params_file', default_value=default_nav2_params,
                                        description='Full path to the nav2 params yaml file to use'))

    # --- Robot State Publisher ---
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file_path]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    ))

    # --- Azure Kinect ROS Driver ---
    ld.add_action(Node(
        package='azure_kinect_ros_driver',
        executable='node',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            'depth_enabled': True,
            'depth_mode': 'NFOV_UNBINNED',
            'color_enabled': True,
            'color_resolution': '720P',
            'fps': 15,
            'point_cloud': False,
            'rgb_point_cloud': False,
            'point_cloud_in_depth_frame': False,
            'synchronized_images_only': True,
            'imu_rate_target': 100,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    ))

    # --- RTAB-Map Visual Odometry ---
    ld.add_action(Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remapping
    ))

    # --- RTAB-Map SLAM ---
    ld.add_action(Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remapping,
    ))

    # --- RViz2 ---
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    ))

    # --- Nav2 nodes (use params_file from azure_kinect/config by default) ---
    nav_nodes = [
        {'package': 'nav2_controller', 'executable': 'controller_server', 'name': 'controller_server'},
        {'package': 'nav2_smoother', 'executable': 'smoother_server', 'name': 'smoother_server'},
        {'package': 'nav2_planner', 'executable': 'planner_server', 'name': 'planner_server'},
        {'package': 'nav2_behaviors', 'executable': 'behavior_server', 'name': 'behavior_server'},
        {'package': 'nav2_bt_navigator', 'executable': 'bt_navigator', 'name': 'bt_navigator'},
        {'package': 'nav2_waypoint_follower', 'executable': 'waypoint_follower', 'name': 'waypoint_follower'},
        # add velocity_smoother here if you want later
    ]

    for nd in nav_nodes:
        ld.add_action(
            Node(
                package=nd["package"],
                executable=nd["executable"],
                name=nd["name"],
                output="screen",
                respawn=LaunchConfiguration("use_respawn"),
                respawn_delay=2.0,
                parameters=configured_params,
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                remappings=rtabmap_remapping + [("cmd_vel", "input_node")],
            )
        )

    # lifecycle manager for nav2
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': LaunchConfiguration('autostart')},
            {'node_names': lifecycle_nodes}
        ]
    ))

    return ld
