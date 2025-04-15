+start(Arm_, ArmLoc_, Robot_, RobotLoc_, Nurse_, NurseLoc_): true <-
    +start(Arm_, ArmLoc_, Robot_, RobotLoc_, Nurse_, NurseLoc_);
    !exist_sample(Robot_, NurseLoc_).

+!exist_sample(Robot_, NurseLoc_) <- !a_navto(Robot_, NurseLoc_).

+!a_navto(Robot_, NurseLoc_): true <-
    a_navto(Robot_, NurseLoc_).

+success_a_navto(Robot_, Loc_): true <-
    !a_approach_nurse(Robot_, Nurse_).

+!a_approach_nurse(Robot_, Nurse_): true <-
    a_approach_nurse(Robot_, Nurse_).

+success_a_approach_nurse(Robot_, Nurse_): true <-
    .send(Nurse_,tell,robot_arrived(Robot_, Room));
    !a_authenticate_nurse(Robot_, Nurse_).

+!a_authenticate_nurse(Robot_, Nurse_): true <-
    a_authenticate_nurse(Robot_, Nurse_).

+success_a_authenticate_nurse(Robot_, Nurse_): true <-
    !a_open_drawer(Robot_).

+!a_open_drawer(Robot_): true <-
    a_open_drawer(Robot_).

+success_a_open_drawer(Robot_): true <-
    .send(Nurse_,tell,robot_opened_drawer(Robot_));
    !a_deposit(Robot_, Nurse_).

+!a_deposit(Robot_, Nurse_): true <-
    a_deposit(Robot_, Nurse_).

+success_a_deposit(Robot_, Nurse_): true <-
    !a_close_drawer(Robot_).

+!a_close_drawer(Robot_): true <-
    a_close_drawer(Robot_).

+success_a_close_drawer(Robot_): true <-
    .send(Nurse_,tell,robot_closed_drawer(Robot_));
    !a_navto(Robot_, Arm_).

+!a_navto(Robot_, Arm_): true <-
    a_navto(Robot_, Loc_).

+success_a_navto(Robot_, Loc_): true <-
    !a_approach_arm(Robot_, Arm_).

+!a_approach_arm(Robot_, Arm_): true <-
    a_approach_arm(Robot_, Arm_).

+success_a_approach_arm(Robot_, Arm_): true <-
    !a_open_drawer(Robot_).

+!a_open_drawer(Robot_): true <-
    a_open_drawer(Robot_).

+success_a_open_drawer(Robot_): true <-
    .send(Arm_,tell,robot_opened_drawer(Robot_));
    !a_pick_up_sample(Arm_, Robot_).

+!a_pick_up_sample(Arm_, Robot_): true <-
    a_pick_up_sample(Arm_, Robot_).

+success_a_pick_up_sample(Arm_, Robot_): true <-
    !a_close_drawer(Robot_).

+!a_close_drawer(Robot_): true <-
    a_close_drawer(Robot_).

+success_a_close_drawer(Robot_): true <-
    .send(Arm_,tell,robot_closed_drawer(Robot_)).
