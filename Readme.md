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



// Navegar entre salas
+!navigate(From, To) : robot_at(Robot, From) & opened_door(From) & opened_door(To) <- 
    .print("Navegando de ", From, " para ", To);
    -robot_at(Robot, From);
    +robot_at(Robot, To).

// Abrir porta
+!open_door(Room, Nurse) : nurse_at(Nurse, Room) & not opened_door(Room) <- 
    .print("Abrindo a porta da sala", Room);
    +opened_door(Room).

// Aproximar-se da enfermeira
+!approach_nurse(Robot, Nurse, Room) : robot_at(Room) & nurse_at(Nurse, Room) & not robot_near_nurse(Robot, Nurse) <- 
    .print("Aproximando-se da enfermeira", Nurse, "na sala", Room);
    +robot_near_nurse(Robot, Nurse).

// Autenticar enfermeira
+!authenticate_nurse(Robot, Nurse) : robot_near_nurse(Robot, Nurse) & not nurse_auth_robot(Robot, Nurse) <- 
    .print("Autenticando com a enfermeira", Nurse);
    +nurse_auth_robot(Robot, Nurse).

// Depositar amostra do enfermeiro para o robô
+!deposit_sample(Nurse, Robot, Room) : robot_drawer(Robot) & nurse_auth_robot(Robot, Nurse) & nurse_has_sample(Nurse) <- 
    .print("Transferindo amostra da enfermeira para o robô.");
    -nurse_has_sample(Nurse);
    +robot_has_sample(Robot).

// Aproximar-se do braço mecânico
+!approach_arm(Robot, Arm, Room) : robot_at(Room) & arm_at(Arm, Room) & not robot_near_arm(Robot, Arm) <- 
    .print("Aproximando-se do braço mecânico", Arm, "na sala", Room);
    +robot_near_arm(Robot, Arm).

// Pegar amostra do robô para o braço mecânico
+!pick_up_sample(Arm, Robot, Room) : robot_has_sample(Robot) & robot_near_arm(Robot, Arm) <- 
    .print("Transferindo amostra do robô para o braço mecânico.");
    -robot_has_sample(Robot);
    +arm_has_sample(Arm).