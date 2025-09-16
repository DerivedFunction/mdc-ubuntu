import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            "use_respawn",
            default_value="False",
            description="Whether to respawn if a node crashes. Applied when composition is disabled.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(os.getcwd(), "config", "rtabmap_slam.rviz"),
            description="Path to the RViz config file",
        )
    )

    # --- RViz2 ---
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            respawn=LaunchConfiguration("use_respawn"),
            respawn_delay=1.0,
            output="screen",
            arguments=["-d", LaunchConfiguration("config")],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        )
    )
    return ld
