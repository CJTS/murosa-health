+start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot): true <-
    +start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot);
    !a_navto(SpotRobot, NurseDisinfectRoom).

+!a_navto(SpotRobot, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    a_navto(SpotRobot, NurseDisinfectRoom).

+success_a_navto(SpotRobot, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    +milestone2;
    !a_patrol_room(SpotRobot, NurseDisinfectRoom).

+!a_patrol_room(SpotRobot, NurseDisinfectRoom): milestone2 <-
    a_patrol_room(SpotRobot, NurseDisinfectRoom).

+success_a_patrol_room(SpotRobot, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone2 <-
    -milestone2;
    +milestone3;
    .send(UvdRobot, tell, milestone2);
    .send(UvdRobot, tell, trigger_a_authorize_disinfect(UvdRobot, SpotRobot));
    !a_authorize_disinfect(UvdRobot, SpotRobot).

+!a_authorize_disinfect(UvdRobot, SpotRobot): milestone3 <-
    a_authorize_disinfect(UvdRobot, SpotRobot).

+success_a_authorize_disinfect(UvdRobot, SpotRobot): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone3 <-
    -milestone3; end.