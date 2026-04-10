source /opt/ros/humble/setup.bash
source install/local_setup.bash
BDI=False REPLAN=True ros2 launch coordinator health.launch.py
