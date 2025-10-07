+stop: start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot) <- 
    -start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot);
    -trigger_a_authenticate_nurse(SpotRobot, NurseDisinfect)[source(SpotRobot)];
    -milestone1[source(SpotRobot)];
    -trigger_a_authorize_patrol(SpotRobot, NurseDisinfect)[source(SpotRobot)];
    -milestone2[source(SpotRobot)];
    -success_a_authorize_patrol(SpotRobot, NurseDisinfect)[source(percept)];
    -stop.

+start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot): true <-
    +start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot).

+trigger_a_approach_nurse(SpotRobot, NurseDisinfect): start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot) <-
    !a_approach_nurse(SpotRobot, NurseDisinfect);
    -trigger_a_approach_nurse(SpotRobot, NurseDisinfect)[source(SpotRobot)].

+!a_approach_nurse(SpotRobot, NurseDisinfect): not low_battery & milestone1 <-
    a_approach_nurse(SpotRobot, NurseDisinfect).

+success_a_approach_nurse(SpotRobot, NurseDisinfect): start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot) & milestone1 <-
    -milestone1[source(SpotRobot)];
    +milestone2;
    !a_authenticate_nurse(SpotRobot, NurseDisinfect).

+!a_authenticate_nurse(SpotRobot, NurseDisinfect): milestone2 <-
    a_authenticate_nurse(SpotRobot, NurseDisinfect).

+success_a_authenticate_nurse(SpotRobot, NurseDisinfect): milestone2 <- 
    -milestone2;
    +milestone3;
    !a_authorize_patrol(SpotRobot, NurseDisinfect).

+!a_authorize_patrol(SpotRobot, NurseDisinfect): milestone3 <-
    a_authorize_patrol(SpotRobot, NurseDisinfect).

+success_a_authorize_patrol(SpotRobot, NurseDisinfect): milestone3 <- 
    -milestone3;
    -start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot);
    -success_a_authorize_patrol(SpotRobot, NurseDisinfect)[source(percept)];
    end.
