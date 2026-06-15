// Stop mission
+stop: start(SpotRobot, NurseRoom, Nurse, UvdRobot) <- 
    -trigger_a_authorize_disinfect(UvdRobot, SpotRobot)[source(SpotRobot)];
    -milestone5[source(SpotRobot)];
    -milestone6;
    -milestone7;
    -success_a_disinfect_room(UvdRobot, NurseRoom);
    -start(SpotRobot, NurseRoom, Nurse, UvdRobot);
    -stop.

// Start disinfect mission
+start(SpotRobot, NurseRoom, Nurse, UvdRobot): true <- 
    +start(SpotRobot, NurseRoom, Nurse, UvdRobot).

// Mission actions
+trigger_a_authorize_disinfect(UvdRobot, SpotRobot): start(SpotRobot, NurseRoom, Nurse, UvdRobot) <- 
    !a_authorize_disinfect(UvdRobot, SpotRobot);
    -trigger_a_authorize_disinfect(UvdRobot, SpotRobot)[source(SpotRobot)].

+!a_authorize_disinfect(UvdRobot, SpotRobot): not low_battery & milestone5 <- 
    a_authorize_disinfect(UvdRobot, SpotRobot).

+success_a_authorize_disinfect(UvdRobot, SpotRobot): start(SpotRobot, NurseRoom, Nurse, UvdRobot) & milestone5 <- 
    -milestone5[source(SpotRobot)];
    +milestone6; 
    !a_navto(UvdRobot, NurseRoom).

+!a_navto(UvdRobot, NurseRoom): not low_battery & milestone6  <-
    a_navto(UvdRobot, NurseRoom).

+success_a_navto(UvdRobot, NurseRoom): start(SpotRobot, NurseRoom, Nurse, UvdRobot) & milestone6 <-
    -milestone6;
    +milestone7;
    !a_disinfect_room(UvdRobot, NurseRoom).

+!a_disinfect_room(UvdRobot, NurseRoom): not low_battery & milestone7 <-
    a_disinfect_room(UvdRobot, NurseRoom).

+success_a_disinfect_room(UvdRobot, NurseRoom): milestone7 <- 
    -milestone7;
    -start(SpotRobot, NurseRoom, Nurse, UvdRobot);
    -success_a_disinfect_room(UvdRobot, NurseRoom);
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