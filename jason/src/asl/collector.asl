+stop: start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
    -milestone_collector1_1;
    -milestone_collector1_2;
    -milestone_collector1_3;
    -milestone_collector1_4;
    -milestone_collector1_5;
    -milestone_collector1_6;
    -milestone_collector1_7;
    -milestone_collector1_8;
    -milestone_collector1_9;
    -milestone_collector1_10;
    -success_a_close_drawer(Collector);
    -start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot);
	stop;
    -stop.

+start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
	!a_navto(Collector, NurseRoom).

+!a_navto(Collector, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & not low_battery <-
	a_navto(Collector, NurseRoom).

+success_a_navto(Collector, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
	+milestone_collector1_1;
	!a_approach_nurse(Collector, Nurse).

+!a_approach_nurse(Collector, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_1 & not low_battery <-
	a_approach_nurse(Collector, Nurse).

+success_a_approach_nurse(Collector, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_1 <-
	-milestone_collector1_1;
	+milestone_collector1_2;
	!a_authenticate_nurse(Collector, Nurse).

+!a_authenticate_nurse(Collector, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_2 & not low_battery <-
	a_authenticate_nurse(Collector, Nurse).

+success_a_authenticate_nurse(Collector, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_2 <-
	-milestone_collector1_2;
	+milestone_collector1_3;
	!a_open_drawer(Collector).

+!a_open_drawer(Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_3 & not low_battery <-
	a_open_drawer(Collector).

+success_a_open_drawer(Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_3 <-
	-milestone_collector1_3;
	+milestone_collector1_4;
	!a_deposit(Nurse, Collector).

+!a_deposit(Nurse, Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_4 & not low_battery <-
	a_deposit(Nurse, Collector).

+success_a_deposit(Nurse, Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_4 <-
	-milestone_collector1_4;
	+milestone_collector1_5;
	!a_close_drawer(Collector).

+!a_close_drawer(Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_5 & not low_battery <-
	a_close_drawer(Collector).

+success_a_close_drawer(Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_5 <-
	-milestone_collector1_5;
	+milestone_collector1_6;
	!a_navto(Collector, ArmRoom).

+!a_navto(Collector, ArmRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_6 & not low_battery <-
	a_navto(Collector, ArmRoom).

+success_a_navto(Collector, ArmRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_6 <-
	-milestone_collector1_6;
	+milestone_collector1_7;
	!a_approach_arm(Collector, Arm).

+!a_approach_arm(Collector, Arm):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_7 & not low_battery <-
	a_approach_arm(Collector, Arm).

+success_a_approach_arm(Collector, Arm):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_7 <-
	-milestone_collector1_7;
	+milestone_collector1_8;
	!a_open_drawer(Collector).

+!a_open_drawer(Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_8 & not low_battery <-
	a_open_drawer(Collector).

+success_a_open_drawer(Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_8 <-
	-milestone_collector1_8;
	+milestone_collector1_9;
	!a_pick_up_sample(Arm, Collector).

+!a_pick_up_sample(Arm, Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_9 & not low_battery <-
	a_pick_up_sample(Arm, Collector).

+success_a_pick_up_sample(Arm, Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_9 <-
	-milestone_collector1_9;
	+milestone_collector1_10;
	!a_close_drawer(Collector).

+!a_close_drawer(Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & not low_battery & milestone_collector1_10<-
	a_close_drawer(Collector).

+success_a_close_drawer(Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_collector1_10 <-
	-milestone_collector1_10;
	-start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot);
	-a_close_drawer(Collector);
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
