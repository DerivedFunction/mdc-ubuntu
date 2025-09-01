#!/bin/bash
# Source your ROS version's default setup
source /opt/ros/humble/setup.bash

# Source the local setup
source install/setup.bash

# colcon searchs the `src` directory for the package.xml file with the 
# package name `mdc_car` and builds it. `package.xml` is located in the `src/mdc_car` directory.
# The following dependencies from http://download.ros.org/schema/package_format3.xsd
# The package will be in the `resource` directory of the workspace
# rclpy: ROS Client Library for the Python language.
# std_msgs: Standard ROS Messages including common message types representing primitive data types
#           and other basic message constructs, such as multiarrays.
# geometry_msgs: provides messages for common geometric primitives such as points, vectors, and poses
colcon build --packages-select mdc_car

# require permission to access the USB port
sudo chmod a+rw /dev/ttyACM0

# Run the mdc_car package using using main_control: main_control.py's main function
ros2 run mdc_car main_control

# SSH into the capstone project device (if needed)
# ssh capstone@capstone-nx.local