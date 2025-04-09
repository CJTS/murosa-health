/* Initial beliefs and rules */
opened_door(room1).
opened_door(room2).
opened_door(room3).

/* Plans */

+start(Arm_, ArmLoc_, Robot_, RobotLoc_, Nurse_, NurseLoc_): true <-
    +nurse_at(Nurse_, NurseLoc_);
    +nurse_has_sample(Nurse_);
    +arm_at(Arm_, ArmLoc_);
    +robot_at(Robot_, RobotLoc_);
    !exist_sample(Robot_, Nurse_).

+!exist_sample(Robot_, Nurse_) <- !a_navto(Robot_, Nurse_);.

/*
exist_sample & !collecting <- collect
collecting & robot_at(Robot_, Loc_) & nurser_at(Nurse_, Loc_) <- approach_nurse
collecting & approached_nurse <- authenticate_nurse
collecting & authenticated_nurse <- open_drawer
collecting & drawer_opened <- collect_sample
collecting & has_sample <- close_drawer
collecting & has_sample & not drawer_opened <- nav_to_arm
collecting & robot_at(Robot_, Loc_) & arm_at(Arm_, Loc_) <- approach_arm
collecting & approached_arm <- authenticate_arm
collecting & authenticated_arm <- open_drawer
collecting & drawer_opened <- deposit_sample
collecting & has_sample <- close_drawer
*/

+!a_navto(Robot_, Nurse_): not collecting(Robot_) & not robot_has_sample(Robot_) & nurse_has_sample(Nurse_) & nurse_at(Nurse_, Loc_) <-
    .print("Navigating");
    +collecting(Robot_);
    a_navto(Robot_, Loc_).

+success_a_navto(Robot_, Loc_): collecting(Robot_) & not robot_has_sample(Robot_) & nurse_at(Nurse_, Loc_) & robot_at(Robot_, Loc2_) <-
    .print("Navigation completed");
    -robot_at(Robot_, Loc2_);
    +robot_at(Robot_, Loc_);
    !a_approach_nurse(Robot_, Nurse_).

+failure_a_navto(Robot_, Loc_): collecting(Robot_) & not robot_has_sample(Robot_) & nurse_at(Nurse_, Loc_) & robot_at(Robot_, Loc2_) <-
    .print("Navigation failed");
    -opened_door(Loc_).

+!a_approach_nurse(Robot_, Nurse_): collecting(Robot_) & robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Approaching");
    a_approach_nurse(Robot_, Nurse_).

+success_a_approach_nurse(Robot_, Nurse_): collecting(Robot_) & not robot_near_nurse(Robot_, Nurse_) & robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Approaching complete");
    +robot_near_nurse(Robot_, Nurse_);
    .send(Nurse_,tell,robot_arrived(Robot_, Room));
    !a_authenticate_nurse(Robot_, Nurse_).

+!a_authenticate_nurse(Robot_, Nurse_): collecting(Robot_) & robot_near_nurse(Robot_, Nurse_) & robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Authenticating");
    a_authenticate_nurse(Robot_, Nurse_).

+success_a_authenticate_nurse(Robot_, Nurse_): collecting(Robot_) & not nurse_authenticated(Robot_, Nurse_) & robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Authenticating complete");
    +nurse_authenticated(Robot_, Nurse_);
    !a_open_drawer(Robot_).

+!a_open_drawer(Robot_): collecting(Robot_) & nurse_authenticated(Robot_, Nurse_) & robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Opening drawer");
    a_open_drawer(Robot_).

+success_a_open_drawer(Robot_): collecting(Robot_) & not drawer_opened(Robot_) & robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Drawer opened complete");
    +drawer_opened(Robot_);
    .send(Nurse_,tell,robot_opened_drawer(Robot_));
    !a_deposit(Robot_, Nurse_).

+!a_deposit(Robot_, Nurse_): collecting(Robot_) & drawer_opened(Robot_) & nurse_authenticated(Robot_, Nurse_) & robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Depositing");
    a_deposit(Robot_, Nurse_).

+success_a_deposit(Robot_, Nurse_): collecting(Robot_) & not robot_has_sample(Robot_) & robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Depositing complete");
    -nurse_authenticated(Robot_, Nurse_);
    -robot_near_nurse(Robot_, Nurse_);
    -nurse_has_sample(Nurse_);
    +robot_has_sample(Robot_);
    !a_close_drawer(Robot_).

+!a_close_drawer(Robot_): collecting(Robot_) & robot_has_sample(Robot_) & robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Closing drawer");
    a_close_drawer(Robot_).

+success_a_close_drawer(Robot_): collecting(Robot_) & robot_has_sample(Robot_) & robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    .print("Drawer closed");
    -drawer_opened(Robot_);
    .send(Nurse_,tell,robot_closed_drawer(Robot_));
    !a_navto(Robot_, Arm_).

+!a_navto(Robot_, Arm_): collecting(Robot_) & robot_has_sample(Robot_) & arm_at(Arm_, Loc_) & robot_at(Robot_, Loc2_) <-
    .print("Navigating to arm");
    a_navto(Robot_, Loc_).

+success_a_navto(Robot_, Loc_): collecting(Robot_) & arm_at(Arm_, Loc_) & robot_at(Robot_, Loc2_) <-
    .print("Navigation completed");
    -robot_at(Robot_, Loc2_);
    +robot_at(Robot_, Loc_);
    !a_approach_arm(Robot_, Arm_).

+!a_approach_arm(Robot_, Arm_): collecting(Robot_) & robot_at(Robot_, Room) & arm_at(Arm_, Room) <-
    .print("Approaching");
    a_approach_arm(Robot_, Arm_).

+success_a_approach_arm(Robot_, Arm_): collecting(Robot_) & not robot_near_arm(Robot_, Arm_) & robot_at(Robot_, Room) & arm_at(Arm_, Room) <-
    .print("Approaching complete");
    +robot_near_arm(Robot_, Arm_);
    .send(Arm_,tell,robot_arrived(Robot_, Room));
    !a_open_drawer(Robot_).

+!a_open_drawer(Robot_): collecting(Robot_) & robot_near_arm(Robot_, Arm_) & robot_at(Robot_, Room) & arm_at(Arm_, Room) <-
    .print("Opening drawer");
    a_open_drawer(Robot_).

+success_a_open_drawer(Robot_): collecting(Robot_) & not drawer_opened(Robot_) & robot_at(Robot_, Room) & arm_at(Arm_, Room) <-
    .print("Drawer opened complete");
    +drawer_opened(Robot_);
    .send(Arm_,tell,robot_opened_drawer(Robot_));
    !a_pick_up_sample(Arm_, Robot_).

+!a_pick_up_sample(Arm_, Robot_): robot_at(Robot_, Room) & arm_at(Arm_, Room) & robot_has_sample(Robot_) <-
    .print("Picking up sample");
    a_pick_up_sample(Arm_, Robot_).

+success_a_pick_up_sample(Arm_, Robot_): collecting(Robot_) & robot_at(Robot_, Room) & arm_at(Arm_, Room) <-
    .print("Drawer opened complete");
    -robot_near_arm(Robot_, Arm_);
    -robot_has_sample(Robot_);
    -collecting(Robot_);
    !a_close_drawer(Robot_).

+!a_close_drawer(Robot_): not collecting(Robot_) & not robot_has_sample(Robot_) & robot_at(Robot_, Room) & arm_at(Arm_, Room) <-
    .print("Closing drawer");
    a_close_drawer(Robot_).

+success_a_close_drawer(Robot_): not collecting(Robot_) & not robot_has_sample(Robot_) & robot_at(Robot_, Room) & arm_at(Arm_, Room) <-
    .print("Drawer closed");
    -drawer_opened(Robot_);
    .send(Arm_,tell,robot_closed_drawer(Robot_)).
