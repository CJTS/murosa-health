+start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot): true <- +start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot).


+trigger_a_authorize_disinfect(UvdRobot, SpotRobot): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <- !a_authorize_disinfect(UvdRobot, SpotRobot).

+!a_authorize_disinfect(UvdRobot, SpotRobot): milestone5 <- a_authorize_disinfect(UvdRobot, SpotRobot).
+success_a_authorize_disinfect(UvdRobot, SpotRobot): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone5 <- -milestone5; +milestone6; !a_navto(UvdRobot, NurseDisinfectRoom).
+!a_navto(UvdRobot, NurseDisinfectRoom): milestone6  <- a_navto(UvdRobot, NurseDisinfectRoom).
+success_a_navto(UvdRobot, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone6 <- -milestone6; +milestone7; !a_disinfect_room(UvdRobot, NurseDisinfectRoom).

+!a_disinfect_room(UvdRobot, NurseDisinfectRoom): milestone7 <- a_disinfect_room(UvdRobot, NurseDisinfectRoom).

+success_a_disinfect_room(UvdRobot, NurseDisinfectRoom): milestone7 <-
    -milestone7;
    end.