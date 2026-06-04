+start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot): true <-
    +start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot).

+trigger_a_authorize_disinfect(UvdRobot, SpotRobot): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    !a_authorize_disinfect(UvdRobot, SpotRobot).

+!a_authorize_disinfect(UvdRobot, SpotRobot): milestone1 <-
    a_authorize_disinfect(UvdRobot, SpotRobot).

+success_a_authorize_disinfect(UvdRobot, SpotRobot): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone1 <-
    -milestone1;
    +milestone2;
    !a_navto(UvdRobot, NurseDisinfectRoom).

+!a_navto(UvdRobot, NurseDisinfectRoom): milestone2 <-
    a_navto(UvdRobot, NurseDisinfectRoom).

+success_a_navto(UvdRobot, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone2 <-
    -milestone2;
    +milestone3;
    !a_disinfect_room(UvdRobot, NurseDisinfectRoom).

+!a_disinfect_room(UvdRobot, NurseDisinfectRoom): milestone3 <-
    a_disinfect_room(UvdRobot, NurseDisinfectRoom).

+success_a_disinfect_room(UvdRobot, NurseDisinfectRoom): milestone3 & start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    -milestone3;
    .send(NurseDisinfect, tell, trigger_nurse_end(NurseDisinfect));
    end.