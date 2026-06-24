+stop: start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
    -milestone_large_delivery_robot1_1;
    -milestone_large_delivery_robot1_2;
    -milestone_large_delivery_robot1_3;
    -milestone_large_delivery_robot1_4;
    -success_a_deliver_resource(LargeDeliveryRobot, NurseRoom);
    -start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot);
    stop;
	-stop.

+start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
	!a_navto(LargeDeliveryRobot, LargeStorage).

+!a_navto(LargeDeliveryRobot, LargeStorage):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & not low_battery <-
	a_navto(LargeDeliveryRobot, LargeStorage).

+success_a_navto(LargeDeliveryRobot, LargeStorage):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
	+milestone_large_delivery_robot1_1;
	!a_request_resource(LargeDeliveryRobot, LargeStorage, LargeResource).

+!a_request_resource(LargeDeliveryRobot, LargeStorage, LargeResource):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_large_delivery_robot1_1 & not low_battery <-
	a_request_resource(LargeDeliveryRobot, LargeStorage, LargeResource).

+success_a_request_resource(LargeDeliveryRobot, LargeStorage, LargeResource):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_large_delivery_robot1_1 <-
	-milestone_large_delivery_robot1_1;
	+milestone_large_delivery_robot1_2;
	!a_pick_resource(LargeDeliveryRobot, LargeStorage, LargeResource).

+!a_pick_resource(LargeDeliveryRobot, LargeStorage, LargeResource):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_large_delivery_robot1_2 & not low_battery <-
	a_pick_resource(LargeDeliveryRobot, LargeStorage, LargeResource).

+success_a_pick_resource(LargeDeliveryRobot, LargeStorage, LargeResource):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_large_delivery_robot1_2 <-
	-milestone_large_delivery_robot1_2;
	+milestone_large_delivery_robot1_3;
	!a_navto(LargeDeliveryRobot, NurseRoom).

+!a_navto(LargeDeliveryRobot, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_large_delivery_robot1_3 & not low_battery <-
	a_navto(LargeDeliveryRobot, NurseRoom).

+success_a_navto(LargeDeliveryRobot, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_large_delivery_robot1_3 <-
	-milestone_large_delivery_robot1_3;
	+milestone_large_delivery_robot1_4;
	!a_deliver_resource(LargeDeliveryRobot, NurseRoom).

+!a_deliver_resource(LargeDeliveryRobot, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & not low_battery & milestone_large_delivery_robot1_4<-
	a_deliver_resource(LargeDeliveryRobot, NurseRoom).

+success_a_deliver_resource(LargeDeliveryRobot, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_large_delivery_robot1_4 <-
	-milestone_large_delivery_robot1_4;
	-start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot);
	-a_deliver_resource(LargeDeliveryRobot, NurseRoom);
	end.

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
