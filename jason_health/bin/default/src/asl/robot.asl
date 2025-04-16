+start(Robot, NurseRoom, Nurse, ArmRoom, Arm): true <- +start(Robot, NurseRoom, Nurse, ArmRoom, Arm); initial_trigger_a_navto(Robot, NurseRoom).

+initial_trigger_a_navto(Robot, NurseRoom): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) <- !a_navto(Robot, NurseRoom).
+!a_navto(Robot, NurseRoom): start(Robot, NurseRoom, Nurse, ArmRoom, Arm) <- a_navto(Robot, NurseRoom).
