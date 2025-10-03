// Stop mission
+stop: start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <- 
    -trigger_a_authorize_disinfect(UvdRobot, SpotRobot)[source(SpotRobot)];
    -milestone5[source(SpotRobot)];
    -milestone6;
    -milestone7;
    -success_a_disinfect_room(UvdRobot, NurseDisinfectRoom);
    -start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot).

// Start disinfect mission
+start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot): true <- 
    +start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot).

// Mission actions
+trigger_a_authorize_disinfect(UvdRobot, SpotRobot): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <- 
    !a_authorize_disinfect(UvdRobot, SpotRobot);
    -trigger_a_authorize_disinfect(UvdRobot, SpotRobot)[source(SpotRobot)].

+!a_authorize_disinfect(UvdRobot, SpotRobot): not low_battery & milestone5 <- 
    a_authorize_disinfect(UvdRobot, SpotRobot).

+success_a_authorize_disinfect(UvdRobot, SpotRobot): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone5 <- 
    -milestone5[source(SpotRobot)];
    +milestone6; 
    !a_navto(UvdRobot, NurseDisinfectRoom).

+!a_navto(UvdRobot, NurseDisinfectRoom): not low_battery & milestone6  <-
    a_navto(UvdRobot, NurseDisinfectRoom).

+success_a_navto(UvdRobot, NurseDisinfectRoom): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) & milestone6 <-
    -milestone6;
    +milestone7;
    !a_disinfect_room(UvdRobot, NurseDisinfectRoom).

+!a_disinfect_room(UvdRobot, NurseDisinfectRoom): not low_battery & milestone7 <-
    a_disinfect_room(UvdRobot, NurseDisinfectRoom).

+success_a_disinfect_room(UvdRobot, NurseDisinfectRoom): milestone7 <- 
    -milestone7;
    -start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot);
    -success_a_disinfect_room(UvdRobot, NurseDisinfectRoom);
    end.

// Charge action
+low_battery_failure(Task): true <- 
    .print("Charging");
    +after_charging(Task);
    +low_battery;
    a_charge.

+success_a_charge: low_battery & after_charging(Task) <-
    .print("Finished charging");
    -after_charging(Task);
    -low_battery;
    !Task.