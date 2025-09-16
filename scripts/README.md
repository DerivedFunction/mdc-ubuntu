# Universal ROS2 Launch Script: `launch.sh`
This script launches various ROS2 nodes for the MDC project.

## Features
  - Launch RTAB-Map SLAM, Navigation2, Kinect driver, and RViz individually or together
  - Flexible argument parsing for custom configurations
  - Logging to timestamped files in ./logs
  - Usage examples and help included

## Usage:
```sh
./launch.sh [options] [args]
```
## Options:
```sh
  -s [args]      Launch RTABMAP (optional args)
  -n [args]      Launch NAV2 (optional config file)
  -K [args]      Launch Kinect only (optional args)
  -r [args]      Launch RViz (optional config file)
  -h, --help     Show help message
```
## Examples:
```sh
./launch.sh                      # Launch everything (default)
./launch.sh -s -n                # Launch RTABMAP and NAV2 only
./launch.sh -s "db_name:=wb map:=false"  # RTABMAP with wb.db, no mapping
```
Log files are saved in `./logs/launch_<timestamp>.log`

## Quick Commands

### Launch the full stack without remapping
```sh
./launch.sh -s "db_name:=wb map:=false" -K -n -r
```

### Launch the mapping mode with Rviz
```sh
./launch.sh -s "db_name:=wb map:=true" -K -r
```