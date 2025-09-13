import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

# --- Interactive DB selection ---
dbname = input("Enter map name: ").strip()
if not dbname:
    dbname = "rtabmap.db"
if not dbname.endswith(".db"):
    dbname += ".db"

deleteDB = input("Delete original db? (y/n): ").strip().lower() == "y"
reuseDB = False
if not deleteDB:  # Only ask if not deleting
    reuseDB = input("Reuse existing db for additional mapping? (y/n): ").strip().lower() == "y"

# --- Paths ---
azure_kinect_ros_driver_pkg_dir = get_package_share_directory('azure_kinect_ros_driver')
rtabmap_ros_pkg_dir = get_package_share_directory('rtabmap_ros')

urdf_file_path = os.path.join(azure_kinect_ros_driver_pkg_dir, 'urdf', 'azure_kinect.urdf.xacro')
rviz_config_file = os.path.join(os.getcwd(), "config", "rtabmap_slam.rviz")

save_location = os.path.join(os.getcwd(), "rtabmap_data")
default_db_path = os.path.join(save_location, dbname)

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
}

# --- Remapping Kinect topics to RTAB-Map ---
rtabmap_remapping = [
    ('rgb/image', '/rgb/image_raw'),
    ('rgb/camera_info', '/rgb/camera_info'),
    ('depth/image', '/depth_to_rgb/image_raw')
]

# --- RTAB-Map DB arguments ---
# Correctly handle arguments vs. parameters
rtabmap_args = []
if deleteDB:
    # This is a command-line argument for the executable
    rtabmap_args.append('--delete_db_on_start')
elif reuseDB:
    # This is a ROS parameter, set it in the dictionary as a boolean
    rtabmap_parameters['Mem/IncrementalMemory'] = 'true'
else:
    # This is a ROS parameter, set it in the dictionary as a boolean
    rtabmap_parameters['Mem/IncrementalMemory'] = 'false'


def generate_launch_description():
    return LaunchDescription([

        # Use simulation clock if needed
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation (Gazebo) clock if true'),

        # Path to DB file
        DeclareLaunchArgument(
            'db_path',
            default_value=default_db_path,
            description='Path to store or load the RTAB-Map database file'
        ),

        # --- Robot State Publisher ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file_path]),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # --- Azure Kinect ROS Driver ---
        Node(
            package='azure_kinect_ros_driver',
            executable='node',
            output='screen',
            respawn=True,
            respawn_delay=3.0,
            parameters=[{
                'depth_enabled': True,
                'depth_mode': 'NFOV_UNBINNED',
                'color_enabled': True,
                'color_resolution': '720P',   # Lowered for performance
                'fps': 30,
                'point_cloud': False,         # Disabled for stability
                'rgb_point_cloud': False,
                'point_cloud_in_depth_frame': False,
                'synchronized_images_only': True,
                'imu_rate_target': 100,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # --- RTAB-Map Visual Odometry ---
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            parameters=[rtabmap_parameters],
            remappings=rtabmap_remapping
        ),

        # --- RTAB-Map SLAM ---
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[rtabmap_parameters],
            remappings=rtabmap_remapping,
            arguments=rtabmap_args   # DB handling only here
        ),

        # --- RViz2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
