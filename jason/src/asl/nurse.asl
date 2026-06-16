+stop: start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
    -milestone_nurse1_1;
    -milestone_nurse1_2;
    -milestone_nurse1_3;
    -milestone_nurse1_4;
    -milestone_nurse1_5;
    -milestone_nurse1_6;
    -milestone_nurse1_7;
    -success_a_authorize_patrol(SpotRobot, Nurse);
    -start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot);
    -stop.

+start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
	!a_navto(Nurse, NurseRoom).

+!a_navto(Nurse, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & not low_battery <-
	a_navto(Nurse, NurseRoom).

+success_a_navto(Nurse, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) <-
	+milestone_nurse1_1;
	!a_collect_sample(Nurse, NurseRoom).

+!a_collect_sample(Nurse, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_nurse1_1 & not low_battery <-
	a_collect_sample(Nurse, NurseRoom).

+success_a_collect_sample(Nurse, NurseRoom):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_nurse1_1 <-
	-milestone_nurse1_1;
	+milestone_nurse1_2;
	!a_approach_nurse(Collector, Nurse).

+!a_approach_nurse(Collector, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_nurse1_2 & not low_battery <-
	a_approach_nurse(Collector, Nurse).

+success_a_approach_nurse(Collector, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_nurse1_2 <-
	-milestone_nurse1_2;
	+milestone_nurse1_3;
	!a_authenticate_nurse(Collector, Nurse).

+!a_authenticate_nurse(Collector, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_nurse1_3 & not low_battery <-
	a_authenticate_nurse(Collector, Nurse).

+success_a_authenticate_nurse(Collector, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_nurse1_3 <-
	-milestone_nurse1_3;
	+milestone_nurse1_4;
	!a_deposit(Nurse, Collector).

+!a_deposit(Nurse, Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_nurse1_4 & not low_battery <-
	a_deposit(Nurse, Collector).

+success_a_deposit(Nurse, Collector):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_nurse1_4 <-
	-milestone_nurse1_4;
	+milestone_nurse1_5;
	!a_approach_nurse(SpotRobot, Nurse).

+!a_approach_nurse(SpotRobot, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_nurse1_5 & not low_battery <-
	a_approach_nurse(SpotRobot, Nurse).

+success_a_approach_nurse(SpotRobot, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_nurse1_5 <-
	-milestone_nurse1_5;
	+milestone_nurse1_6;
	!a_authenticate_nurse(SpotRobot, Nurse).

+!a_authenticate_nurse(SpotRobot, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_nurse1_6 & not low_battery <-
	a_authenticate_nurse(SpotRobot, Nurse).

+success_a_authenticate_nurse(SpotRobot, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_nurse1_6 <-
	-milestone_nurse1_6;
	+milestone_nurse1_7;
	!a_authorize_patrol(SpotRobot, Nurse).

+!a_authorize_patrol(SpotRobot, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & not low_battery & milestone_nurse1_7<-
	a_authorize_patrol(SpotRobot, Nurse).

+success_a_authorize_patrol(SpotRobot, Nurse):
	start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot) & milestone_nurse1_7 <-
	-milestone_nurse1_7;
	-start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot);
	-a_authorize_patrol(SpotRobot, Nurse);
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
