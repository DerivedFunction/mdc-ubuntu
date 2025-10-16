# Return to workspace if in scripts folder
if [ "$(basename "$PWD")" = "scripts" ]; then
  echo "In scripts directory → moving up..."
  cd ..
fi
# --- Launch ---
source install/setup.bash
ros2 launch rviz2.launch.py config:=config/nav2_slam.rviz