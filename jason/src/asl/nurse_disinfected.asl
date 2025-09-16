+start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot): true <- +start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot).

+trigger_a_authenticate_nurse(SpotRobot, NurseDisinfect): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    !a_authenticate_nurse(SpotRobot, NurseDisinfect);
    -trigger_a_authenticate_nurse(SpotRobot, NurseDisinfect)[source(SpotRobot)].

+!a_authenticate_nurse(SpotRobot, NurseDisinfect): milestone2 <-
    a_authenticate_nurse(SpotRobot, NurseDisinfect).

+success_a_authenticate_nurse(SpotRobot, NurseDisinfect): milestone2 <- 
    -milestone2[source(SpotRobot)].

+trigger_a_authorize_patrol(SpotRobot, NurseDisinfect): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    !a_authorize_patrol(SpotRobot, NurseDisinfect);
    -trigger_a_authorize_patrol(SpotRobot, NurseDisinfect)[source(SpotRobot)].

+!a_authorize_patrol(SpotRobot, NurseDisinfect): milestone3 <-
    a_authorize_patrol(SpotRobot, NurseDisinfect).

+success_a_authorize_patrol(SpotRobot, NurseDisinfect): milestone3 <- 
    -milestone3[source(SpotRobot)];
    -start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot);
    -success_a_authorize_patrol(SpotRobot, NurseDisinfect)[source(percept)];
    end.
