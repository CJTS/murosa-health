+stop: start(SpotRobot, NurseRoom, Nurse, UvdRobot) <- 
    -start(SpotRobot, NurseRoom, Nurse, UvdRobot);
    -trigger_a_authenticate_nurse(SpotRobot, Nurse)[source(SpotRobot)];
    -milestone1[source(SpotRobot)];
    -trigger_a_authorize_patrol(SpotRobot, Nurse)[source(SpotRobot)];
    -milestone2[source(SpotRobot)];
    -success_a_authorize_patrol(SpotRobot, Nurse)[source(percept)];
    -stop.

+start(SpotRobot, NurseRoom, Nurse, UvdRobot): true <-
    +start(SpotRobot, NurseRoom, Nurse, UvdRobot).

+trigger_a_approach_nurse(SpotRobot, Nurse): start(SpotRobot, NurseRoom, Nurse, UvdRobot) <-
    !a_approach_nurse(SpotRobot, Nurse);
    -trigger_a_approach_nurse(SpotRobot, Nurse)[source(SpotRobot)].

+!a_approach_nurse(SpotRobot, Nurse): not low_battery & milestone1 <-
    a_approach_nurse(SpotRobot, Nurse).

+success_a_approach_nurse(SpotRobot, Nurse): start(SpotRobot, NurseRoom, Nurse, UvdRobot) & milestone1 <-
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
    -start(SpotRobot, NurseRoom, Nurse, UvdRobot);
    -success_a_authorize_patrol(SpotRobot, Nurse)[source(percept)];
    end.
