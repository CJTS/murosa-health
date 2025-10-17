// Stop mission
+stop: start(SpotRobot, NurseRoom, Nurse, UvdRobot) <- 
    -start(SpotRobot, NurseRoom, Nurse, UvdRobot);
    -milestone1;
    -milestone1;
    -milestone2;
    -milestone3;
    -milestone4;
    -milestone5; 
    -success_a_authorize_disinfect(UvdRobot, SpotRobot);
    -stop.

// Start disinfect mission
+start(SpotRobot, NurseRoom, Nurse, UvdRobot): true <-
    +start(SpotRobot, NurseRoom, Nurse, UvdRobot);
    !a_navto(SpotRobot, NurseRoom).

// Mission actions
+!a_navto(SpotRobot, NurseRoom): not low_battery & start(SpotRobot, NurseRoom, Nurse, UvdRobot) <-
    a_navto(SpotRobot, NurseRoom).

+success_a_navto(SpotRobot, NurseRoom): start(SpotRobot, NurseRoom, Nurse, UvdRobot) <-
    +milestone1;
    .send(Nurse, tell, milestone1);
    .send(Nurse, tell, trigger_a_approach_nurse(SpotRobot, Nurse));
    !a_approach_nurse(SpotRobot, Nurse).

+!a_approach_nurse(SpotRobot, Nurse): not low_battery & milestone1 <-
    a_approach_nurse(SpotRobot, Nurse).

+success_a_approach_nurse(SpotRobot, Nurse): start(SpotRobot, NurseRoom, Nurse, UvdRobot) & milestone1 <-
    -milestone1;
    +milestone2;
    !a_authenticate_nurse(SpotRobot, Nurse).

+!a_authenticate_nurse(SpotRobot, Nurse): not low_battery & milestone2 <-
    a_authenticate_nurse(SpotRobot, Nurse).

+success_a_authenticate_nurse(SpotRobot, Nurse): start(SpotRobot, NurseRoom, Nurse, UvdRobot) & milestone2 <-
    -milestone2;
    +milestone3;
    !a_authorize_patrol(SpotRobot, Nurse).

+!a_authorize_patrol(SpotRobot, Nurse): not low_battery & milestone3 <-
    a_authorize_patrol(SpotRobot, Nurse).

+success_a_authorize_patrol(SpotRobot, Nurse): start(SpotRobot, NurseRoom, Nurse, UvdRobot) & milestone3 <-
    -milestone3;
    +milestone4;
    !a_patrol_room(SpotRobot, NurseRoom).

+!a_patrol_room(SpotRobot, NurseRoom): not low_battery & milestone4 <-
    a_patrol_room(SpotRobot, NurseRoom).

+success_a_patrol_room(SpotRobot, NurseRoom): start(SpotRobot, NurseRoom, Nurse, UvdRobot) & milestone4 <-
    -milestone4;
    +milestone5;
    .send(UvdRobot, tell, milestone5);
    .send(UvdRobot, tell, trigger_a_authorize_disinfect(UvdRobot, SpotRobot));
    !a_authorize_disinfect(UvdRobot, SpotRobot).

+!a_authorize_disinfect(UvdRobot, SpotRobot): not low_battery & milestone5 <-
    a_authorize_disinfect(UvdRobot, SpotRobot).

+success_a_authorize_disinfect(UvdRobot, SpotRobot): start(SpotRobot, NurseRoom, Nurse, UvdRobot) & milestone5 <-
    -milestone5; 
    -start(SpotRobot, NurseRoom, Nurse, UvdRobot);
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
