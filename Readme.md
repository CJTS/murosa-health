# Build

To run the project first you need to build the docker image

```$ podman build -t health_ros .```

# Run

Then you can use python script to run the simulation

```$ python runSimulations.py ```

or you can run a single execution with

```$ podman-compose -f experiment_trials.yaml up health ```

## podman help

### Delete pod and containers

```podman rm -f -a && podman pod rm -f -a```

### Restart machine
```podman machine reset```
```podman machine init```
```podman machine start```

### Increase keys

```podman machine ssh```
```sudo sysctl -w kernel.keys.maxkeys=2000000```

## ROS help

Publish and read topics

```ros2 topic pub /move_base/move std_msgs/String "data: Hello World"```
```ros2 topic echo /move_base/move```

# WSL

## Build

```colcon build --symlink-install```

## Run

To run the program you need to run ROS Bridge and Jason before.

### ROS Bridge
```ros2 launch rosbridge_server rosbridge_websocket_launch.xml```

### Jason
```cd jason_health```
```./gradlew run```

Then source the build, set the ENV variables REPLAN and PROBLEM_RATE, run the coordinator, and run one of the cases
```source install/local_setup.bash```
```ros2 launch coordinator planning.launch.py```

### Health
```ros2 launch murosa_plan_health planning.launch.py```

### Patrol
```ros2 launch murosa_plan_patrol planning.launch.py```
