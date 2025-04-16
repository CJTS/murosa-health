+start(Arm_, ArmLoc_, Robot_, RobotLoc_, Nurse_, NurseLoc_): true <-
    +start(Arm_, ArmLoc_, Robot_, RobotLoc_, Nurse_, NurseLoc_).
    
+robot_arrived(Robot_, Room_): robot_at(Robot_, Room1_) & nurse_at(Nurse_, Room_) <-
    -robot_at(Robot_, Room1_);
    +robot_at(Robot_, Room_);
    !a_authenticate_nurse(Robot_, Nurse_).

+!a_authenticate_nurse(Robot_, Nurse_): robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Authenticating");
    a_authenticate_nurse(Robot_, Nurse_).

+success_a_authenticate_nurse(Robot_, Nurse_): not nurse_authenticated(Robot_, Nurse_) & robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Authenticating complete");
    +nurse_authenticated(Robot_, Nurse_).

+robot_opened_drawer(Robot_): nurse_authenticated(Robot_, Nurse_) & robot_at(Robot_, Room1_) & nurse_at(Nurse_, Room_) <-
    +drawer_opened(Robot_);
    !a_deposit(Robot_, Nurse_).

+!a_deposit(Robot_, Nurse_): robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Depositing");
    a_deposit(Robot_, Nurse_).

+success_a_deposit(Robot_, Nurse_): robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Deposit complete");
    -nurse_has_sample(Nurse_).

+robot_closed_drawer(Robot_): nurse_authenticated(Robot_, Nurse_) & robot_at(Robot_, Room1_) & nurse_at(Nurse_, Room_) <-
    -drawer_opened(Robot_);
    -nurse_authenticated(Robot_, Nurse_).

/*

-!G[error(no_relevant)] : teacher(T) <-
    .send(T, askHow, { +!G }, Plans);
    .add_plan(Plans);
    !G.

Não sei o que
-!G[error(no_relevant)] : true <-
    .print("no_relevant ", G).

Não tenho como fazer
-!G[error(no_applicable)] : true <-
    .print("no_applicable ", G).

start(arm1, room3, robot1, room2, nurse1, room1).
!a_deposit(robot1, nurse1).
 */