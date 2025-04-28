+start(Patrol, Base, Room1): not low_battery <-
    .print("Starting with battery");
    +start(Patrol, Base, Room1);
    !move(Patrol, Room1).

+!move(Patrol, Room1): start(Patrol, Base, Room1) <-
    .print("moving to room with battery");
    move(Patrol, Room1).

+success_move(Patrol, Room1): start(Patrol, Base, Room1) <-
    .print("Finished moving to room with battery");
    +milestone0;
    !move(Patrol, Base).

+!move(Patrol, Base): milestone0 <- 
    .print("Moving to base with battery");
    move(Patrol, Base).

+success_move(Patrol, Base): start(Patrol, Base, Room1) & milestone0 <-
    .print("Finished moving to base with battery");
    -milestone0;
    -start(Patrol, Base, Room1);
    end.

+failure_move(Patrol, _): start(Patrol, Base, Room1) <- 
    .print("Failed to move");
    -start(Patrol, Base, Room1);
    -milestone0;
    +low_battery;
    end.

+start(Patrol, Base, Room1): low_battery <-
    .print("Starting without battery");
    +start(Patrol, Base);
    -start(Patrol, Base, Room1);
    !charge(Patrol).

+!charge(Patrol): start(Patrol, Base) <- 
    .print("Charging without battery");
    charge(Patrol).

+success_charge(Patrol): start(Patrol, Base) <-
    .print("Finished charging without battery");
    -start(Patrol, Base);
    -low_battery;
    end.