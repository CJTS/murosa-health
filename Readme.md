# Build

To run the project first you need to build the docker image

```$ docker build -t planner_nodes .```

# Run

Then you can use python script to run the simulation

```$ python runSimulations.py ```

or you can run a single execution with

```$ podman-compose -f experiment_trials.yaml up run ```

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