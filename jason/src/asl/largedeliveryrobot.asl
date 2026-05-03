+stop: start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom) <- 
    -trigger_a_navto(SmallDeliveryRobot, SmallStorage)[source(SmallDeliveryRobot)];
    -trigger_a_request_resource(SmallDeliveryRobot, SmallStorage, SmallResource)[source(SmallDeliveryRobot)];
    -trigger_a_pick_resource(SmallDeliveryRobot, SmallStorage, SmallResource)[source(SmallDeliveryRobot)];
    -trigger_a_navto(SmallDeliveryRobot, LargeDeliveryRobot)[source(SmallDeliveryRobot)];
    -trigger_a_deliver_resource(SmallDeliveryRobot, LargeDeliveryRobot)[source(SmallDeliveryRobot)];
    -trigger_a_navto(LargeStorage, LargeResource)[source(LargeStorage)];
    -trigger_a_request_resource(LargeStorage, LargeResource, NurseRoom)[source(LargeStorage)];
    -trigger_a_pick_resource(LargeStorage, LargeResource, NurseRoom)[source(LargeStorage)];
    -trigger_a_navto(LargeStorage, LargeDeliveryRobot)[source(LargeStorage)];
    -trigger_a_deliver_resource(LargeStorage, LargeDeliveryRobot)[source(LargeStorage)];
    -milestone1;
    -milestone2;
    -milestone3;
    -milestone4;
    -milestone5;
    -milestone5[source(SmallDeliveryRobot)];
    -milestone6;
    -milestone7;
    -milestone8;
    -milestone9;
    -milestone10;
    -success_a_deliver_resource(LargeStorage, LargeDeliveryRobot);
    -start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom);
    -stop.
+start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom): true <- +start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom); .
+trigger_a_navto(LargeStorage, LargeResource): start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom) <- !a_navto(LargeStorage, LargeResource); -trigger_a_navto(LargeStorage, LargeResource)[source(SmallDeliveryRobot)].
+!a_navto(LargeStorage, LargeResource): not low_battery & milestone5 <- a_navto(LargeStorage, LargeResource).
+success_a_navto(LargeStorage, LargeResource): start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom) & milestone5 <- -milestone5[source(SmallDeliveryRobot)]; +milestone6; !a_request_resource(LargeStorage, LargeResource, NurseRoom).
+!a_request_resource(LargeStorage, LargeResource, NurseRoom): not low_battery & milestone6 <- a_request_resource(LargeStorage, LargeResource, NurseRoom).
+success_a_request_resource(LargeStorage, LargeResource, NurseRoom): start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom) & milestone6 <- -milestone6; +milestone7; !a_pick_resource(LargeStorage, LargeResource, NurseRoom).
+!a_pick_resource(LargeStorage, LargeResource, NurseRoom): not low_battery & milestone7 <- a_pick_resource(LargeStorage, LargeResource, NurseRoom).
+success_a_pick_resource(LargeStorage, LargeResource, NurseRoom): start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom) & milestone7 <- -milestone7; +milestone8; !a_navto(LargeStorage, LargeDeliveryRobot).
+!a_navto(LargeStorage, LargeDeliveryRobot): not low_battery & milestone8 <- a_navto(LargeStorage, LargeDeliveryRobot).
+success_a_navto(LargeStorage, LargeDeliveryRobot): start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom) & milestone8 <- -milestone8; +milestone9; !a_deliver_resource(LargeStorage, LargeDeliveryRobot).
+!a_deliver_resource(LargeStorage, LargeDeliveryRobot): not low_battery & milestone9 <- a_deliver_resource(LargeStorage, LargeDeliveryRobot).
+success_a_deliver_resource(LargeStorage, LargeDeliveryRobot): milestone9 <- -milestone9; -start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom); -a_deliver_resource(LargeStorage, LargeDeliveryRobot); end.
+low_battery_failure(Task): true <- .print("Charging"); +after_charging(Task); +low_battery; a_charge.
+success_a_charge: low_battery & after_charging(Task) <- .print("Finished charging"); -after_charging(Task); -low_battery; !Task.