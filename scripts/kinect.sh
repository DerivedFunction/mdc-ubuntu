# Return to workspace if in scripts folder
if [ "$(basename "$PWD")" = "scripts" ]; then
  echo "In scripts directory → moving up..."
  cd ..
fi

# --- Launch ---
cd Azure_Kinect_ROS_Driver
source install/setup.bash
export DISPLAY=:0
ros2 launch launch/kinect.launch.py