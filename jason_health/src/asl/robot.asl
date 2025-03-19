/* Initial beliefs and rules */
opened_door(room1).
opened_door(room2).
opened_door(room3).

/* Plans */

+!exist_sample <- !a_navto(Robot_, Loc_).

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

+!a_navto(Robot_, Loc_): true <-
    a_navto(Robot_, Loc_).

+!success_navto(Robot_, Loc_): true <-
    -robot_at(Robot_, _);
    +robot_at(Robot_, Loc_).

+start: true <- !exist_sample.