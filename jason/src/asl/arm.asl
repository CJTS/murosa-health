+start(Nurse, NurseRoom, Robot, ArmRoom, Arm): true <- +start(Nurse, NurseRoom, Robot, ArmRoom, Arm).

+trigger_a_approach_arm(Robot, Arm): start(Nurse, NurseRoom, Robot, ArmRoom, Arm) <- !a_approach_arm(Robot, Arm).
+!a_approach_arm(Robot, Arm): milestone6 <- a_approach_arm(Robot, Arm).
+success_a_approach_arm(Robot, Arm): milestone6 <- -milestone6.
+trigger_a_pick_up_sample(Arm, Robot): start(Nurse, NurseRoom, Robot, ArmRoom, Arm) <- !a_pick_up_sample(Arm, Robot).
+!a_pick_up_sample(Arm, Robot): milestone8 <- a_pick_up_sample(Arm, Robot).
+success_a_pick_up_sample(Arm, Robot): milestone8 <- -milestone8; end.