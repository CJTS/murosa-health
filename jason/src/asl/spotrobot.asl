+start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot): true <-
    +start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot);
    !a_navto(SpotRobot, NurseDisinfectRoom).

+!a_navto(SpotRobot, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    a_navto(SpotRobot, NurseDisinfectRoom).

+success_a_navto(SpotRobot, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    +milestone1;
    !a_patrol_room(SpotRobot, NurseDisinfectRoom).

+!a_patrol_room(SpotRobot, NurseDisinfectRoom): milestone1 <-
    a_patrol_room(SpotRobot, NurseDisinfectRoom).

+success_a_patrol_room(SpotRobot, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone1 <-
    -milestone1;
    +milestone2;
    .send(UvdRobot, tell, milestone1);
    .send(UvdRobot, tell, trigger_a_authorize_disinfect(UvdRobot, SpotRobot));
    !a_authorize_disinfect(UvdRobot, SpotRobot).

+!a_authorize_disinfect(UvdRobot, SpotRobot): milestone2 <-
    a_authorize_disinfect(UvdRobot, SpotRobot).

+success_a_authorize_disinfect(UvdRobot, SpotRobot): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone2 <-
    -milestone2; end.