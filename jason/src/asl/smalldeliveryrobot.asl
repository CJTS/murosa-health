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
+start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom): true <- +start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom); !a_navto(SmallDeliveryRobot, SmallStorage).
+initial_trigger_a_navto(SmallDeliveryRobot, SmallStorage): start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom) <- !a_navto(SmallDeliveryRobot, SmallStorage).
+!a_navto(SmallDeliveryRobot, SmallStorage): not low_battery & start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom) <- a_navto(SmallDeliveryRobot, SmallStorage).
+success_a_navto(SmallDeliveryRobot, SmallStorage): start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom) <- +milestone1; -initial_trigger_a_navto(SmallDeliveryRobot, SmallStorage); !a_request_resource(SmallDeliveryRobot, SmallStorage, SmallResource).
+!a_request_resource(SmallDeliveryRobot, SmallStorage, SmallResource): not low_battery & milestone1 <- a_request_resource(SmallDeliveryRobot, SmallStorage, SmallResource).
+success_a_request_resource(SmallDeliveryRobot, SmallStorage, SmallResource): start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom) & milestone1 <- -milestone1; +milestone2; !a_pick_resource(SmallDeliveryRobot, SmallStorage, SmallResource).
+!a_pick_resource(SmallDeliveryRobot, SmallStorage, SmallResource): not low_battery & milestone2 <- a_pick_resource(SmallDeliveryRobot, SmallStorage, SmallResource).
+success_a_pick_resource(SmallDeliveryRobot, SmallStorage, SmallResource): start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom) & milestone2 <- -milestone2; +milestone3; !a_navto(SmallDeliveryRobot, LargeDeliveryRobot).
+!a_navto(SmallDeliveryRobot, LargeDeliveryRobot): not low_battery & milestone3 <- a_navto(SmallDeliveryRobot, LargeDeliveryRobot).
+success_a_navto(SmallDeliveryRobot, LargeDeliveryRobot): start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom) & milestone3 <- -milestone3; +milestone4; !a_deliver_resource(SmallDeliveryRobot, LargeDeliveryRobot).
+!a_deliver_resource(SmallDeliveryRobot, LargeDeliveryRobot): not low_battery & milestone4 <- a_deliver_resource(SmallDeliveryRobot, LargeDeliveryRobot).
+success_a_deliver_resource(SmallDeliveryRobot, LargeDeliveryRobot): start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom) <- .send(LargeStorage, tell, milestone5); .send(LargeStorage, tell, trigger_a_navto(LargeStorage, LargeResource)); -start(SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, NurseRoom); -a_deliver_resource(SmallDeliveryRobot, LargeDeliveryRobot); end.
+low_battery_failure(Task): true <- .print("Charging"); +after_charging(Task); +low_battery; a_charge.
+success_a_charge: low_battery & after_charging(Task) <- .print("Finished charging"); -after_charging(Task); -low_battery; !Task.