source install/setup.bash
# export DISPLAY=:0
if [[ "$1" == "m" ]]; then
    ros2 launch launch/slam_rtabmap.launch.py
else
    ros2 launch launch/nav2_rtabmap.launch.py
fi
# Try https://github.com/introlab/rtabmap/wiki/Kinect-mapping