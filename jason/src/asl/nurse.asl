+start(Nurse, NurseRoom, Robot, ArmRoom, Arm): true <- +start(Nurse, NurseRoom, Robot, ArmRoom, Arm).

+trigger_a_authenticate_nurse(Robot, Nurse): start(Nurse, NurseRoom, Robot, ArmRoom, Arm) <- !a_authenticate_nurse(Robot, Nurse).
+!a_authenticate_nurse(Robot, Nurse): milestone1 <- a_authenticate_nurse(Robot, Nurse).
+success_a_authenticate_nurse(Robot, Nurse): milestone1 <- -milestone1.
+trigger_a_deposit(Nurse, Robot): start(Nurse, NurseRoom, Robot, ArmRoom, Arm) <- !a_deposit(Nurse, Robot).
+!a_deposit(Nurse, Robot): milestone3 <- a_deposit(Nurse, Robot).
+success_a_deposit(Nurse, Robot): milestone3 <- -milestone3; end.

+stop: start(Nurse, NurseRoom, SpotRobot, UvdRobot) <- 
    -start(Nurse, NurseRoom, SpotRobot, UvdRobot);
    -trigger_a_authenticate_nurse(SpotRobot, Nurse)[source(SpotRobot)];
    -milestone1[source(SpotRobot)];
    -trigger_a_authorize_patrol(SpotRobot, Nurse)[source(SpotRobot)];
    -milestone2[source(SpotRobot)];
    -success_a_authorize_patrol(SpotRobot, Nurse)[source(percept)];
    -stop.

+start(Nurse, NurseRoom, SpotRobot, UvdRobot): true <-
    +start(Nurse, NurseRoom, SpotRobot, UvdRobot).

+trigger_a_approach_nurse(SpotRobot, Nurse): start(Nurse, NurseRoom, SpotRobot, UvdRobot) <-
    !a_approach_nurse(SpotRobot, Nurse);
    -trigger_a_approach_nurse(SpotRobot, Nurse)[source(SpotRobot)].

+!a_approach_nurse(SpotRobot, Nurse): not low_battery & milestone1 <-
    a_approach_nurse(SpotRobot, Nurse).

+success_a_approach_nurse(SpotRobot, Nurse): start(Nurse, NurseRoom, SpotRobot, UvdRobot) & milestone1 <-
    -milestone1[source(SpotRobot)];
    +milestone2;
    !a_authenticate_nurse(SpotRobot, Nurse).

+!a_authenticate_nurse(SpotRobot, Nurse): milestone2 <-
    a_authenticate_nurse(SpotRobot, Nurse).

+success_a_authenticate_nurse(SpotRobot, Nurse): milestone2 <- 
    -milestone2;
    +milestone3;
    !a_authorize_patrol(SpotRobot, Nurse).

+!a_authorize_patrol(SpotRobot, Nurse): milestone3 <-
    a_authorize_patrol(SpotRobot, Nurse).

+success_a_authorize_patrol(SpotRobot, Nurse): milestone3 <- 
    -milestone3;
    -start(Nurse, NurseRoom, SpotRobot, UvdRobot);
    -success_a_authorize_patrol(SpotRobot, Nurse)[source(percept)];
    end.
