#!/usr/bin/env bash
# Universal Launch script to launch each ROS nodes separately or all together

# --- Help ---
print_help() {
  cat <<EOF
Usage: $0 [options] [args]

Options:
  -s [args]      Launch RTABMAP (optional args)
  -n [args]      Launch NAV2 (optional config file)
  -K [args]      Launch Kinect only (optional args)
  -r [args]      Launch RViz (optional config file)
  -h, --help     Show this help message

Examples:
  $0                              Launch everything (default if no args)
  $0 -s -n                        Launch RTABMAP and NAV2 only
  $0 -s "db_name:=wb map:=false"  Launch RTABMAP with wb.db and no mapping
EOF
############################################################
# README: Universal ROS2 Launch Script
# ------------------------------------
# This script launches various ROS2 nodes for the MDC project.
#
# Features:
#   - Launch RTAB-Map SLAM, Navigation2, Kinect driver, and RViz individually or together
#   - Flexible argument parsing for custom configurations
#   - Logging to timestamped files in ./logs
#   - Usage examples and help included
#
# Usage:
#   ./launch.sh [options] [args]
#
# Options:
#   -s [args]      Launch RTABMAP (optional args)
#   -n [args]      Launch NAV2 (optional config file)
#   -K [args]      Launch Kinect only (optional args)
#   -r [args]      Launch RViz (optional config file)
#   -h, --help     Show help message
#
# Examples:
#   ./launch.sh                      # Launch everything (default)
#   ./launch.sh -s -n                # Launch RTABMAP and NAV2 only
#   ./launch.sh -s "db_name:=wb map:=false"  # RTABMAP with wb.db, no mapping
#
# Log files are saved in ./logs/launch_<timestamp>.log
#
# For more details, see the project documentation.
############################################################
}

kill_all(){
  
}
PIDS=() # array to hold processes
# --- Logging setup ---
mkdir -p logs
LOGFILE="logs/launch_$(date +'%Y-%m-%d_%H-%M-%S').log"
echo "Logging to $LOGFILE"
exec > >(tee -a "$LOGFILE") 2>&1

# Default launch flags + args
launch_rtabmap=false; rtabmap_args=""
launch_nav2=false;    nav2_args=""
launch_kinect=false;  kinect_args=""
launch_rviz=false;    rviz_args=""

# If no args, launch everything
if [ $# -eq 0 ]; then
  echo "No arguments → launching everything..."
  launch_rtabmap=true
  launch_nav2=true
  launch_kinect=true
  launch_rviz=true
else
  # Parse
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -s)
        launch_rtabmap=true
        shift
        # Capture optional db arg
        if [[ $# -gt 0 && ! "$1" =~ ^- ]]; then
          rtabmap_args="$1"
          shift
        fi
        ;;
      -n)
        launch_nav2=true
        shift
        if [[ $# -gt 0 && ! "$1" =~ ^- ]]; then
          nav2_args="$1"
          shift
        fi
        ;;
      -K)
        launch_kinect=true
        shift
        if [[ $# -gt 0 && ! "$1" =~ ^- ]]; then
          kinect_args="$1"
          shift
        fi
        ;;
      -r)
        launch_rviz=true
        shift
        if [[ $# -gt 0 && ! "$1" =~ ^- ]]; then
          rviz_args="$1"
          shift
        fi
        ;;
      -h|--help)
        print_help
        exit 0
        ;;
      *)
        echo "Unknown option: $1"
        print_help
        exit 1
        ;;
    esac
  done
fi

# Return to workspace if in scripts folder
if [ "$(basename "$PWD")" = "scripts" ]; then
  echo "In scripts directory → moving up..."
  cd ..
fi

# --- Launch ---
if $launch_rtabmap; then
  if [ -n "$rtabmap_args" ]; then
    echo "Launching RTABMAP with config: $rtabmap_args"
    ros2 launch launch/rtabmap.launch.py $rtabmap_args &
    PIDS+=($!)
  else
    echo "Launching RTABMAP..."
    ros2 launch launch/rtabmap.launch.py &
    PIDS+=($!)
  fi
fi

if $launch_nav2; then
  if [ -n "$nav2_args" ]; then
    echo "Launching NAV2 with config: $nav2_args"
    ros2 launch launch/nav2.launch.py $nav2_args &
    PIDS+=($!)
  else
    echo "Launching NAV2..."
    ros2 launch launch/nav2.launch.py &
    PIDS+=($!)
  fi
fi

if $launch_kinect; then
  echo "Launching Kinect with config: $kinect_args"
  cd Azure_Kinect_ROS_Driver
  source install/setup.bash
  ros2 launch launch/kinect.launch.py $kinect_args &
  PIDS+=($!)
  cd ..
fi

# --- RViz arguments ---
if $launch_rtabmap; then
  rviz_args="config:=config/rtabmap_slam.rviz"
fi
if $launch_nav2; then
  # Use Nav2 config if Nav2 is launched
  rviz_args="config:=config/nav2_slam.rviz"
fi

if $launch_rviz; then
  if [ -n "$rviz_args" ]; then
    echo "Launching RViz with config: $rviz_args"
    ros2 launch launch/rviz2.launch.py $rviz_args &
    PIDS+=($!)
  else
    echo "Launching RViz with default config (rtabmap_slam.rviz)..."
    ros2 launch launch/rviz2.launch.py &
    PIDS+=($!)
  fi
fi
