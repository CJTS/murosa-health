export ROS_LOG_LEVEL=debug
source /opt/ros/humble/setup.bash
source install/local_setup.bash
BDI=True PROBLEM_RATE=100 ros2 launch agents health.launch.py
