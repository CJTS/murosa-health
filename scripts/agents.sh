source /opt/ros/humble/setup.bash
source install/local_setup.bash
BDI=False PROBLEM_RATE=0 ros2 launch agents health.launch.py
