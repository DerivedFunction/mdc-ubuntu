#!/bin/bash
# Source your ROS version's default setup
source /opt/ros/humble/setup.bash
# Clean the directory
make clean

# Build only the packages we want
colcon build --packages-select mdc_car laptop

# Allow all scripts to be execuable
chmod +x *.sh
