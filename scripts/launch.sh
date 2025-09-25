# Return to workspace if in scripts folder
if [ "$(basename "$PWD")" = "scripts" ]; then
  echo "In scripts directory → moving up..."
  cd ..
fi

# --- Launch ---
cd Azure_Kinect_ROS_Driver
source install/setup.bash
cd ..
ros2 launch Azure_Kinect_ROS_Driver/launch/nav2_rtabmap.launch.py