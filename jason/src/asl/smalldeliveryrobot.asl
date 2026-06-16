+stop: start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
    -milestone_small_delivery_robot1_1;
    -milestone_small_delivery_robot1_2;
    -milestone_small_delivery_robot1_3;
    -milestone_small_delivery_robot1_4;
    -success_a_deliver_resource(SmallDeliveryRobot, NurseRoom);
    -start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot);
    -stop.

+start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
	!a_navto(SmallDeliveryRobot, SmallStorage).

+!a_navto(SmallDeliveryRobot, SmallStorage):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & not low_battery <-
	a_navto(SmallDeliveryRobot, SmallStorage).

+success_a_navto(SmallDeliveryRobot, SmallStorage):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
	+milestone_small_delivery_robot1_1;
	!a_request_resource(SmallDeliveryRobot, SmallStorage, SmallResource).

+!a_request_resource(SmallDeliveryRobot, SmallStorage, SmallResource):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_small_delivery_robot1_1 & not low_battery <-
	a_request_resource(SmallDeliveryRobot, SmallStorage, SmallResource).

+success_a_request_resource(SmallDeliveryRobot, SmallStorage, SmallResource):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_small_delivery_robot1_1 <-
	-milestone_small_delivery_robot1_1;
	+milestone_small_delivery_robot1_2;
	!a_pick_resource(SmallDeliveryRobot, SmallStorage, SmallResource).

+!a_pick_resource(SmallDeliveryRobot, SmallStorage, SmallResource):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_small_delivery_robot1_2 & not low_battery <-
	a_pick_resource(SmallDeliveryRobot, SmallStorage, SmallResource).

+success_a_pick_resource(SmallDeliveryRobot, SmallStorage, SmallResource):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_small_delivery_robot1_2 <-
	-milestone_small_delivery_robot1_2;
	+milestone_small_delivery_robot1_3;
	!a_navto(SmallDeliveryRobot, NurseRoom).

+!a_navto(SmallDeliveryRobot, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_small_delivery_robot1_3 & not low_battery <-
	a_navto(SmallDeliveryRobot, NurseRoom).

+success_a_navto(SmallDeliveryRobot, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_small_delivery_robot1_3 <-
	-milestone_small_delivery_robot1_3;
	+milestone_small_delivery_robot1_4;
	!a_deliver_resource(SmallDeliveryRobot, NurseRoom).

+!a_deliver_resource(SmallDeliveryRobot, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & not low_battery & milestone_small_delivery_robot1_4<-
	a_deliver_resource(SmallDeliveryRobot, NurseRoom).

+success_a_deliver_resource(SmallDeliveryRobot, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_small_delivery_robot1_4 <-
	-milestone_small_delivery_robot1_4;
	-start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot);
	-a_deliver_resource(SmallDeliveryRobot, NurseRoom);
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
