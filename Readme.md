# Requirements
This project runs in a Ubuntu 22.04 OS. ROS2 (Humble) is required (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Java is also needed if using BDI Agents.

## ROS Bridge

```
sudo apt install ros-humble-rosbridge-server
```

## Build
To build the ros2 packages run the following command

```
colcon build --symlink-install
```

## Config

Then source the build, set the ENV variables REPLAN and PROBLEM_RATE, run the coordinator, and run one of the cases
```
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

## Run

To run the program you need to run ROS Bridge and Jason before.

### With BDI
If you are using the BDI version, start rosbridge and Jason

#### ROS Bridge
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

#### Jason
```
cd jason
./gradlew run
```

### With Front
If you want to see the robots moving in the grid, start the front (Angular) project. Is uses Node 25 and Angular 20. RosBridge is also needed for this, see running with bdi to know how to start rosbridge. You have to be in the front folder

```
cd front
```

#### First time
It is necessary to install the angular project dependencies to start it. Do it by running:

```
npm i
```

#### Running
In the front folder, run:

```
npm start
```


### Healthcare case
In one terminal, start the coordinator nodes (coordinator and planner)

```
BDI=False REPLAN=True ros2 launch coordinator health.launch.py
```

Then, start the env and agent nodes.

```
BDI=False PROBLEM_RATE=0 ros2 launch agents health.launch.py
```
