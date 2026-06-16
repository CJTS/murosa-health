+stop: start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
    -milestone_uvd1_1;
    -milestone_uvd1_2;
    -success_a_disinfect_room(UvdRobot, NurseRoom);
    -start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot);
    -stop.

+start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
	!a_authorize_disinfect(UvdRobot, SpotRobot).

+!a_authorize_disinfect(UvdRobot, SpotRobot):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & not low_battery <-
	a_authorize_disinfect(UvdRobot, SpotRobot).

+success_a_authorize_disinfect(UvdRobot, SpotRobot):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
	+milestone_uvd1_1;
	!a_navto(UvdRobot, NurseRoom).

+!a_navto(UvdRobot, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_uvd1_1 & not low_battery <-
	a_navto(UvdRobot, NurseRoom).

+success_a_navto(UvdRobot, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_uvd1_1 <-
	-milestone_uvd1_1;
	+milestone_uvd1_2;
	!a_disinfect_room(UvdRobot, NurseRoom).

+!a_disinfect_room(UvdRobot, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & not low_battery & milestone_uvd1_2<-
	a_disinfect_room(UvdRobot, NurseRoom).

+success_a_disinfect_room(UvdRobot, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_uvd1_2 <-
	-milestone_uvd1_2;
	-start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot);
	-a_disinfect_room(UvdRobot, NurseRoom);
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
