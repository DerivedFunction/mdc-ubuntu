#!/bin/bash
# This is the inital setup for the Kinect DK
# source: git clone -b humble https://github.com/microsoft/Azure_Kinect_ROS_Driver.git
# We will need these if it is not installed: 
# pip3 install xacro
# sudo apt install ros-humble-joint-state-publisher

# Source your ROS version's default setup
source /opt/ros/humble/setup.bash

# Go to the Azure Kinect ROS Driver director
cd Azure_Kinect_ROS_Driver

# Make runAzure.sh executable
chmod +x *.sh

# Build the package in the directory
colcon build
