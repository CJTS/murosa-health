+start(Nurse, NurseRoom, Robot, ArmRoom, Arm): true <- +start(Nurse, NurseRoom, Robot, ArmRoom, Arm).

+trigger_a_authenticate_nurse(Robot, Nurse): start(Nurse, NurseRoom, Robot, ArmRoom, Arm) <- !a_authenticate_nurse(Robot, Nurse).
+!a_authenticate_nurse(Robot, Nurse): milestone1 <- a_authenticate_nurse(Robot, Nurse).
+success_a_authenticate_nurse(Robot, Nurse): milestone1 <- -milestone1.
+trigger_a_deposit(Nurse, Robot): start(Nurse, NurseRoom, Robot, ArmRoom, Arm) <- !a_deposit(Nurse, Robot).
+!a_deposit(Nurse, Robot): milestone3 <- a_deposit(Nurse, Robot).
+success_a_deposit(Nurse, Robot): milestone3 <- -milestone3; end.
