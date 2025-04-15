+robot_arrived(Robot_, Room_): robot_at(Robot_, Room1_) & arm_at(Arm_, Room_) <-
    -robot_at(Robot_, Room1_);
    +robot_at(Robot_, Room_);
    !a_pick_up_sample(Arm_, Robot_).

+!a_pick_up_sample(Arm_, Robot_): robot_at(Robot_, Room_) & arm_at(Arm_, Room_) <-
    .print("a_pick_up_sample");
    a_pick_up_sample(Arm_, Robot_).

+success_a_pick_up_sample(Arm_, Robot_): robot_at(Robot_, Room) & arm_at(Arm_, Room) <-
    .print("a_pick_up_sample complete");
    +arm_has_sample(Arm_);
    -nurse_has_sample(_).

+robot_closed_drawer(Robot_): robot_at(Robot_, Room1_) & amr_arm(Arm_, Room_) <-
    -drawer_opened(Robot_).