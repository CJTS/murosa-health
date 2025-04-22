+start(Robot, NurseRoom, Nurse, ArmRoom, Arm): true <- +start(Robot, NurseRoom, Nurse, ArmRoom, Arm).

+trigger_a_authenticate_nurse(Robot, Nurse): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) <- !a_authenticate_nurse(Robot, Nurse).
+!a_authenticate_nurse(Robot, Nurse): milestone1 <- a_authenticate_nurse(Robot, Nurse).
+success_a_authenticate_nurse(Robot, Nurse): milestone1 <- -milestone1.
+trigger_a_deposit(Nurse, Robot): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) <- !a_deposit(Nurse, Robot).
+!a_deposit(Nurse, Robot): milestone3 <- a_deposit(Nurse, Robot).
+success_a_deposit(Nurse, Robot): milestone3 <- -milestone3; end.
