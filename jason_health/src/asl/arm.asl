+start(Robot, NurseRoom, Nurse, ArmRoom, Arm): true <- +start(Robot, NurseRoom, Nurse, ArmRoom, Arm).

+trigger_a_approach_arm(Robot, Arm): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) <- !a_approach_arm(Robot, Arm).
+!a_approach_arm(Robot, Arm): milestone6 <- a_approach_arm(Robot, Arm).
+success_a_approach_arm(Robot, Arm): milestone6 <- -milestone6.
+trigger_a_pick_up_sample(Arm, Robot): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) <- !a_pick_up_sample(Arm, Robot).
+!a_pick_up_sample(Arm, Robot): milestone8 <- a_pick_up_sample(Arm, Robot).
+success_a_pick_up_sample(Arm, Robot): milestone8 <- -milestone8; end.