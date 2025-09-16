import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# --- Paths ---
save_location = os.path.join(os.getcwd(), "rtabmap_data")

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
    "database_path": LaunchConfiguration("db_path"),  # <- now arg-driven
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
    ("rgb/image", "/rgb/image_raw"),
    ("rgb/camera_info", "/rgb/camera_info"),
    ("depth/image", "/depth_to_rgb/image_raw"),
]


def generate_launch_description():
    ld = LaunchDescription()

    # Common launch arguments
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "db_name",
            default_value="rtabmap.db",
            description="Database filename (will be stored in rtabmap_data/)",
        )
    )

    # Compose full db_path from db_name
    def set_db_path(context, *args, **kwargs):
        db_name = LaunchConfiguration("db_name").perform(context)
        if not db_name.endswith(".db"):
            db_name += ".db"
        db_path = os.path.join(save_location, db_name)
        return [DeclareLaunchArgument("db_path", default_value=db_path)]

    ld.add_action(OpaqueFunction(function=set_db_path))

    ld.add_action(
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            description="Whether to respawn if a node crashes. Applied when composition is disabled.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "log_level", default_value="info", description="log level"
        )
    )

    # --- RTAB-Map Visual Odometry ---
    ld.add_action(
        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            output="screen",
            parameters=[rtabmap_parameters],
            remappings=rtabmap_remapping,
        )
    )

    # --- RTAB-Map SLAM ---
    ld.add_action(
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            output="screen",
            parameters=[rtabmap_parameters],
            respawn=LaunchConfiguration("use_respawn"),
            respawn_delay=1.0,
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration("log_level"),
            ],
            remappings=rtabmap_remapping,
        )
    )
    return ld
