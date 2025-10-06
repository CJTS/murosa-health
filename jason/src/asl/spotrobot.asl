// Stop mission
+stop: start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot) <- 
    -start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot);
    -milestone1;
    -milestone2;
    -milestone3;
    -milestone4;
    -milestone5;
    -milestone6; 
    -success_a_authorize_disinfect(UvdRobot, SpotRobot);
    -stop.

// Start disinfect mission
+start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot): true <-
    +start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot);
    !a_navto(SpotRobot, NurseDisinfectRoom).

// Mission actions
+!a_navto(SpotRobot, NurseDisinfectRoom): not low_battery & start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot) <-
    a_navto(SpotRobot, NurseDisinfectRoom).

+success_a_navto(SpotRobot, NurseDisinfectRoom): start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot) <-
    +milestone1;
    !a_open_door(SpotRobot, NurseDisinfectRoom).

+!a_open_door(SpotRobot, NurseDisinfectRoom): not low_battery & milestone1 <-
    a_open_door(SpotRobot, NurseDisinfectRoom).

-!a_open_door(SpotRobot, NurseDisinfectRoom): start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot) <-
    !a_open_door(SpotRobot, NurseDisinfectRoom).

+success_a_open_door(SpotRobot, NurseDisinfectRoom): start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot) & milestone1 <-
    -milestone1;
    +milestone2;
    .send(NurseDisinfect, tell, milestone2);
    .send(NurseDisinfect, tell, trigger_a_approach_nurse(SpotRobot, NurseDisinfect));
    !a_approach_nurse(SpotRobot, NurseDisinfect).

+!a_approach_nurse(SpotRobot, NurseDisinfect): not low_battery & milestone2 <-
    a_approach_nurse(SpotRobot, NurseDisinfect).

+success_a_approach_nurse(SpotRobot, NurseDisinfect): start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot) & milestone2 <-
    -milestone2;
    +milestone3;
    !a_authenticate_nurse(SpotRobot, NurseDisinfect).

+!a_authenticate_nurse(SpotRobot, NurseDisinfect): not low_battery & milestone3 <-
    a_authenticate_nurse(SpotRobot, NurseDisinfect).

+success_a_authenticate_nurse(SpotRobot, NurseDisinfect): start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot) & milestone3 <-
    -milestone3;
    +milestone4;
    !a_authorize_patrol(SpotRobot, NurseDisinfect).

+!a_authorize_patrol(SpotRobot, NurseDisinfect): not low_battery & milestone4 <-
    a_authorize_patrol(SpotRobot, NurseDisinfect).

+success_a_authorize_patrol(SpotRobot, NurseDisinfect): start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot) & milestone4 <-
    -milestone4;
    +milestone5;
    !a_patrol_room(SpotRobot, NurseDisinfectRoom).

+!a_patrol_room(SpotRobot, NurseDisinfectRoom): not low_battery & milestone5 <-
    a_patrol_room(SpotRobot, NurseDisinfectRoom).

+success_a_patrol_room(SpotRobot, NurseDisinfectRoom): start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot) & milestone5 <-
    -milestone5;
    +milestone6;
    .send(UvdRobot, tell, milestone5);
    .send(UvdRobot, tell, trigger_a_authorize_disinfect(UvdRobot, SpotRobot));
    !a_authorize_disinfect(UvdRobot, SpotRobot).

+!a_authorize_disinfect(UvdRobot, SpotRobot): not low_battery & milestone6 <-
    a_authorize_disinfect(UvdRobot, SpotRobot).

+success_a_authorize_disinfect(UvdRobot, SpotRobot): start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot) & milestone6 <-
    -milestone6; 
    -start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot);
    -success_a_authorize_disinfect(UvdRobot, SpotRobot);
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
