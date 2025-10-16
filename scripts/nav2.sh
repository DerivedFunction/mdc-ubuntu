# Return to workspace if in scripts folder
if [ "$(basename "$PWD")" = "scripts" ]; then
  echo "In scripts directory → moving up..."
  cd ..
fi
# --- Launch ---
source install/setup.bash

ros2 launch launch/nav2.launch.py