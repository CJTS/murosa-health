+start(Robot, NurseRoom, Nurse, ArmRoom, Arm): true <- +start(Robot, NurseRoom, Nurse, ArmRoom, Arm); !a_navto(Robot, NurseRoom).

+!a_navto(Robot, NurseRoom): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) <- a_navto(Robot, NurseRoom).
+success_a_navto(Robot, NurseRoom): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) <- +milestone1; !a_approach_nurse(Robot, Nurse).
+!a_approach_nurse(Robot, Nurse): milestone1 <- a_approach_nurse(Robot, Nurse).
+success_a_approach_nurse(Robot, Nurse): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) & milestone1 <- -milestone1; +milestone2; .send(nurse1, tell, milestone1); .send(nurse1, tell, trigger_a_authenticate_nurse(Robot, Nurse)); !a_authenticate_nurse(Robot, Nurse).
+!a_authenticate_nurse(Robot, Nurse): milestone2 <- a_authenticate_nurse(Robot, Nurse).
+success_a_authenticate_nurse(Robot, Nurse): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) & milestone2 <- -milestone2; +milestone3; !a_open_drawer(Robot).
+!a_open_drawer(Robot): milestone3 <- a_open_drawer(Robot).
+success_a_open_drawer(Robot): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) & milestone3 <- -milestone3; +milestone4; .send(nurse1, tell, milestone3); .send(nurse1, tell, trigger_a_deposit(Nurse, Robot)); !a_deposit(Nurse, Robot).
+!a_deposit(Nurse, Robot): milestone4 <- a_deposit(Nurse, Robot).
+success_a_deposit(Nurse, Robot): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) & milestone4 <- -milestone4; +milestone5; !a_close_drawer(Robot).
+!a_close_drawer(Robot): milestone5 <- a_close_drawer(Robot).
+success_a_close_drawer(Robot): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) & milestone5 <- -milestone5; +milestone6; !a_navto(Robot, ArmRoom).
+!a_navto(Robot, ArmRoom): milestone6 <- a_navto(Robot, ArmRoom).
+success_a_navto(Robot, ArmRoom): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) & milestone6 <- -milestone6; +milestone7; .send(Arm, tell, milestone6); .send(Arm, tell, trigger_a_approach_arm(Robot, Arm)); !a_approach_arm(Robot, Arm).
+!a_approach_arm(Robot, Arm): milestone7 <- a_approach_arm(Robot, Arm).
+success_a_approach_arm(Robot, Arm): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) & milestone7 <- -milestone7; +milestone8; !a_open_drawer(Robot).
+!a_open_drawer(Robot): milestone8 <- a_open_drawer(Robot).
+success_a_open_drawer(Robot): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) & milestone8 <- -milestone8; +milestone9; .send(Arm, tell, milestone8); .send(Arm, tell, trigger_a_pick_up_sample(Arm, Robot)); !a_pick_up_sample(Arm, Robot).
+!a_pick_up_sample(Arm, Robot): milestone9 <- a_pick_up_sample(Arm, Robot).
+success_a_pick_up_sample(Arm, Robot): milestone9 <- -milestone9; end.