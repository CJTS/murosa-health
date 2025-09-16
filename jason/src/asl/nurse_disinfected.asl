+start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot): true <- +start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot).

+trigger_a_authenticate_nurse(SpotRobot, NurseDisinfect): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <- !a_authenticate_nurse(SpotRobot, NurseDisinfect).
+!a_authenticate_nurse(SpotRobot, NurseDisinfect): milestone2 <- a_authenticate_nurse(SpotRobot, NurseDisinfect).
+success_a_authenticate_nurse(SpotRobot, NurseDisinfect): milestone2 <- -milestone2.
+trigger_a_authorize_patrol(SpotRobot, NurseDisinfect): start(NurseDisinfect, NurseDisinfectRoom, SpotRobot, UvdRobot) <- !a_authorize_patrol(SpotRobot, NurseDisinfect).
+!a_authorize_patrol(SpotRobot, NurseDisinfect): milestone3 <-  a_authorize_patrol(SpotRobot, NurseDisinfect).
+success_a_authorize_patrol(SpotRobot, NurseDisinfect): milestone3 <- -milestone3; end.
