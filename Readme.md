# Requirements
This project runs in WLS2 using a Ubuntu 22.04 OS. ROS2 is required (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Java is also needed if using BDI Agents.

## Build

```colcon build --symlink-install```

## Run

To run the program you need to run ROS Bridge and Jason before.

### ROS Bridge
```ros2 launch rosbridge_server rosbridge_websocket_launch.xml```

### Jason
```cd jason```
```./gradlew run```

Then source the build, set the ENV variables REPLAN and PROBLEM_RATE, run the coordinator, and run one of the cases
```source /opt/ros/humble/setup.bash```
```source install/local_setup.bash```

### Health
```ros2 launch coordinator health.launch.py```
```ros2 launch murosa_plan health.launch.py```

### Patrol
```ros2 launch coordinator patrol.launch.py```
```ros2 launch murosa_plan patrol.launch.py```

### Desinfect
BDI not implemented and needed.

```BDI=False REPLAN=True ros2 launch coordinator disinfect.launch.py```
```BDI=False PROBLEM_RATE=0 ros2 launch murosa_plan disinfect.launch.py```
