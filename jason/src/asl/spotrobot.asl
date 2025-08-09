+start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot): true <-
    +start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot);
    !a_navto(SpotRobot, NurseDisinfectRoom).

+!a_navto(SpotRobot, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    a_navto(SpotRobot, NurseDisinfectRoom).

+success_a_navto(SpotRobot, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    +milestone1;
    !a_open_door(SpotRobot, NurseDisinfectRoom).

+!a_open_door(SpotRobot, NurseDisinfectRoom): milestone1 <-
    a_open_door(SpotRobot, NurseDisinfectRoom).

+success_a_open_door(SpotRobot, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone1 <-
    -milestone1;
    +milestone2;
    !a_approach_nurse(SpotRobot, NurseDisinfect).

+!a_approach_nurse(SpotRobot, NurseDisinfect): milestone2 <-
    a_approach_nurse(SpotRobot, NurseDisinfect).

+success_a_approach_nurse(SpotRobot, NurseDisinfect): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone2 <-
    -milestone2;
    +milestone3;
    .send(NurseDisinfect, tell, milestone2);
    .send(NurseDisinfect, tell, trigger_a_authenticate_nurse(SpotRobot, NurseDisinfect));
    !a_authenticate_nurse(SpotRobot, NurseDisinfect).

+!a_authenticate_nurse(SpotRobot, NurseDisinfect): milestone3 <-
    a_authenticate_nurse(SpotRobot, NurseDisinfect).

+success_a_authenticate_nurse(SpotRobot, NurseDisinfect): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone3 <-
    -milestone3;
    +milestone4;
    .send(NurseDisinfect, tell, milestone3);
    .send(NurseDisinfect, tell, trigger_a_authorize_patrol(SpotRobot, NurseDisinfect));
    !a_authorize_patrol(SpotRobot, NurseDisinfect).

+!a_authorize_patrol(SpotRobot, NurseDisinfect): milestone4 <-
    a_authorize_patrol(SpotRobot, NurseDisinfect).

+success_a_authorize_patrol(SpotRobot, NurseDisinfect): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone4 <-
    -milestone4;
    +milestone5;
    !a_patrol_room(SpotRobot, NurseDisinfectRoom).

+!a_patrol_room(SpotRobot, NurseDisinfectRoom): milestone5 <-
    a_patrol_room(SpotRobot, NurseDisinfectRoom).

+success_a_patrol_room(SpotRobot, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone5 <-
    -milestone5;
    +milestone6;
    .send(UvdRobot, tell, milestone5);
    .send(UvdRobot, tell, trigger_a_authorize_disinfect(UvdRobot, SpotRobot));
    !a_authorize_disinfect(UvdRobot, SpotRobot).

+!a_authorize_disinfect(UvdRobot, SpotRobot): milestone6 <-
    a_authorize_disinfect(UvdRobot, SpotRobot).

+success_a_authorize_disinfect(UvdRobot, SpotRobot): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone6 <-
    -milestone6; end.


