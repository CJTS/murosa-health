/* Initial beliefs and rules */
opened_door(room1).
opened_door(room2).
opened_door(room3).

/* Initial goals */
+!goal <- !pickup_and_deliver_sample(room1).

/* Plans */

+!pickup_and_deliver_sample(Room): robot_at(Robot, RobotRoom) <-
    .print("Robot ", Robot, " collecting sample in room ", Room);
    !m_approach_nurse(Robot_, Nurse_);
    !m_pick_sample(Robot_, Nurse_).

+!m_approach_nurse(Robot_, Nurse_): nurse_at(Nurse, Room) & opened_door(Room) <-
    .print("m_approach_nurse");
    !a_approach_nurse(Robot_, Nurse_);
    !a_authenticate_nurse(Robot_, Nurse_).

+!m_approach_nurse(Robot_, Nurse_): nurse_at(Nurse_, Room) & not opened_door(Room) <-
    .print("m_approach_nurse");
    !a_open_door(Nurse_, Room);
    !a_approach_nurse(Robot_, Nurse_);
    !a_authenticate_nurse(Robot_, Nurse_).

+!m_pick_sample(Robot_, Nurse_): robot_at(Robot_, Room) & nurse_at(Nurse_, Room) & nurse_has_sample(Nurse_) <-
    .print("m_pick_sample");
    !a_deposit(Nurse_, Robot_).

+!a_open_door(Nurse_, Room_): not opened_door(Room_) <-
    +opened_door(Room_);
    a_open_door(Nurse_, Room_).

+!a_approach_nurse(Robot_, Nurse_): robot_at(Robot_, Room) & nurse_at(Nurse_, Room) <-
    +robot_near_nurse(Robot_, Nurse_);
    a_approach_nurse(Robot_, Nurse_).

+!a_authenticate_nurse(Robot_, Nurse_): robot_near_nurse(Robot_, Nurse_) <-
    +nurse_authenticated(Robot_, Nurse_);
    a_authenticate_nurse(Robot_, Nurse_).

+!a_deposit(Nurse_, Robot_): robot_at(Robot_, Room) & nurse_at(Nurse_, Room) & nurse_has_sample(Nurse_) <-
    -nurse_authenticated(Robot_, Nurse_);
    -robot_near_nurse(Robot_, Nurse_);
    -nurse_has_sample(Nurse_);
    +robot_has_sample(Robot_);
    a_deposit(Nurse_, Robot_).