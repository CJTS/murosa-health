/* Initial beliefs and rules */
robot_at(r, room2).
nurse_at(n, room1).
arm_at(a, room3).
nurse_has_sample(n).
opened_door(room1).
opened_door(room2).
opened_door(room3).

/* Plans */

+!has_sample <- !pickup_and_deliver_sample(room1).

+!pickup_and_deliver_sample(Room): robot_at(Robot, RobotRoom) <-
    .print("Robot ", Robot, " collecting sample in room ", Room);
    !m_approach_nurse(Robot_, Nurse_, Arm_);
    !m_pick_sample(Robot_, Nurse_);
    !m_approach_arm(Robot_, Nurse_, Arm_);
    !m_unload_sample(Robot_, Arm_).

+!m_approach_nurse(Robot_, Nurse_, Arm_): nurse_at(Nurse, Room) & opened_door(Room) <-
    .print("m_approach_nurse");
    !a_navto(Robot_, Room);
    !a_approach_nurse(Robot_, Nurse_);
    !a_authenticate_nurse(Robot_, Nurse_).

+!m_pick_sample(Robot_, Nurse_): robot_at(Robot_, Room) & nurse_at(Nurse_, Room) & nurse_has_sample(Nurse_) <-
    .print("m_pick_sample");
    !a_open_drawer(Robot_);
    !a_deposit(Nurse_, Robot_);
    !a_close_drawer(Robot_).

+!m_approach_arm(Robot_, Nurse_, Arm_): arm_at(Arm_, Room) & opened_door(Room) <-
    .print("m_approach_arm");
    !a_navto(Robot_, Room);
    !a_approach_arm(Robot_, Arm_).

+!m_unload_sample(Robot_, Arm_): robot_at(Robot_, Room) & arm_at(Arm_, Room) & robot_has_sample(Robot_) <- 
    .print("m_unload_sample");
    !a_open_drawer(Robot_);
    !a_pick_up_sample(Arm_, Robot_);
    !a_close_drawer(Robot_).

+!a_navto(Robot_, Loc_): true <-
    a_navto(Robot_, Loc_).

+!result_navto(Robot_, Loc_): true <-
    -robot_at(Robot_, _);
    +robot_at(Robot_, Loc_).

+start: true <- !has_sample.