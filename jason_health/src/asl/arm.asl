/* Initial goals */
+!goal <- !m_unload_sample(r, a).

/* Plans */

+!m_unload_sample(Robot_, Arm_): arm_at(Arm_, Room) <-
    .print("m_unload_sample");
    !a_pick_up_sample(Arm_, Robot_).

+!a_pick_up_sample(Arm_, Robot_): arm_at(Arm_, Room) <-
    -robot_near_arm(Robot_, Arm_);
    -robot_has_sample(Robot_);
    +arm_has_sample(Arm_);
    a_pick_up_sample(Arm_, Robot_).
