+start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot): true <-
    +start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot).


+trigger_nurse_end(NurseDisinfect): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    end.


+trigger_a_clean_room(NurseDisinfect, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    !a_clean_room(NurseDisinfect, NurseDisinfectRoom).

+!a_clean_room(NurseDisinfect, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    a_clean_room(NurseDisinfect, NurseDisinfectRoom).

+success_a_clean_room(NurseDisinfect, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    +milestone_clean.

+trigger_a_authorize_patrol(SpotRobot, NurseDisinfect): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone_clean <-
    -milestone_clean;
    !a_authorize_patrol(SpotRobot, NurseDisinfect).

+!a_authorize_patrol(SpotRobot, NurseDisinfect): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    a_authorize_patrol(SpotRobot, NurseDisinfect).

+success_a_authorize_patrol(SpotRobot, NurseDisinfect): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <-
    end.