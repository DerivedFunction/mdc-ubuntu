#!/bin/bash
# Source the your ROS version's default setup
source /opt/ros/humble/setup.bash

# Source the local setup:
source install/setup.bash

# colcon searchs the `src` directory for the package.xml file with the
# package name `laptop` and builds it. `package.xml` is located in the `src/laptop` directory.
# The following dependencies from http://download.ros.org/schema/package_format3.xsd
# The package will be in the `resource` directory of the workspace
# rclpy: ROS Client Library for the Python language.
# std_msgs: Standard ROS Messages including common message types representing primitive data types
#           and other basic message constructs, such as multiarrays.
# geometry_msgs: provides messages for common geometric primitives such as points, vectors, and poses
colcon build --packages-select laptop

# Run from laptop's package controller: laptop_controller.py's main function
ros2 run laptop input_node controller

# Connect to the capstone project device (if needed)
# ssh capstone@capstone-nx.local controller
