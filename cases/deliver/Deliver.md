# Deliver Goods / Equipment

## Description (the textual description of the robotic mission)
When required, a robot must collect the required resources in the storage and deliver them to the requesting agent in a specified location.

In the collection phase, the robot must go to the storage places where the resources can be found.

The order to be followed is defined by the estimation of waiting time summed up with the path to be run by the robot.

Once the robot reaches one storage and it is time to request the resource, the robot sends a message to the storage with the precise specification of the requested resources and waits until the resources have been retrieved.

Once retrieved, the delivery phase begins.

In this phase, the robot will make as many runs as necessary to take all the resources to the specified location.

If the battery of the assigned robot runs low (10\%) in the collection phase, the robot goes back to the recharging station and assigns the mission to another robot.

If the battery of the assigned robot runs low (30\%) in the delivery phase, the robot must return the resource to a checkpoint and assign the remaining task to another robot, which will know where the resource is positioned.

In case of failure to return the resource to a checkpoint, an alert must be triggered and the report sent to the sector manager.

When multiple items are required from different storages, multiple robots can be assigned to parallel collect-deliver tasks to reduce the time to finish the mission.

Examples of goods/ equipments are: (i) sterile medical equipment Logistics, which should be transported from sterile facilities to use destinations, and (ii) clean linens, which should periodically be moved from the laundry to storage rooms close to where it should be used.

## Multirobot
Multi-robot teams (some of the robots have distinct behaviours)

## Robot Features
Parameter adaptability (whether the robots can adapt their behaviour by changing some parameters, e.g., their speed), Component adaptability (whether the robots can adapt their behaviour by adding or removing components)

## Technical Capabilities
Robot capabilities (e.g., perception and interpretive, robot task abilities, actions and envisioning capabilities), Interaction capabilities (e.g., physical interaction with the environment, social interaction, cognitive interaction with other information systems)

## Operational Capabilities
Usability, Re-usability, Reliability

## Incompleteness (Caused by lack of knowledge about parts of the internal robot or external environment state).
Will 30% battery be enough to reach checkpoints from any location?

## Variability Space (Caused by the size of the variability space that adaption functions need to handle).
the movement and possible change of load/unload checkpoints that affects also timing of the mission

## Decentralization & Coordination (Caused by decision making by robots in teams or swarms, of which the effects may not be predictable).
when the battery is low the robot should trigger a replacing robot or in case of multiple storage loading requirements, assign part of the job to another robot.

## Execution Context (Caused by the inherent unpredictability of execution contexts, exacerbated by the complexity of the cyber-physical environment).
hospital environment is unpredictable. load/unload areas might continuously change make sure not to load not sterile material

## Changing capabilities (Caused by dynamicity of capabilities in the robotic system).
adding a new robot to the system or reassigning the task to a new one

## Types of adaptation (self* properties)
self-organization (if it maintains, improves or restores a safety property following certain actions), self-scaling (if it maintains or improves a property during the occurrence of a set of actions)

## Adaptation concerns, constraints and other factors
Timing constraints

## If any technical capability was checked, please explain why it was checked.
robots need to move to defined targets and trigger another robot when needed (to reassign tasks or complete the task in parallel)

## If you check any operational capability please explain why it was checked
the task need to be don in the minimum time possible and perform the same task several times

## If any robot feature was checked, please explain why it was checked.
robots need to adjust detect right places to load/unload and trigger another robot when needed (to reassign tasks or complete the task in parallel)

## If you check any adaptation type please explain why it was checked
recharging battery and adding/removing extra robots to the system