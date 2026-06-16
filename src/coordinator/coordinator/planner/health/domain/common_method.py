#!/usr/bin/env python
"""
File Description: Hospital methods file. All the methods for Hospital planning domain are defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from coordinator.planner.ipyhop import Methods

# ******************************************        Method Definitions      ****************************************** #
methods = Methods()

def full_mission(
        state,
        small_box_delivery_robot_,
        small_box_storage_,
        small_resource_,
        large_box_delivery_robot_,
        large_box_storage_,
        large_resource_,
        room_,
        nurse_,
        robot_,
        arm_,
        spotrobot_,
        uvdrobot_
    ):
    return [
        ('a_navto', nurse_, room_),
        ('m_deliver_resource_task', small_box_delivery_robot_, small_box_storage_, small_resource_, large_box_delivery_robot_, large_box_storage_, large_resource_, room_),
        ('m_pickup_and_deliver_sample', nurse_, room_, robot_, arm_),
        ('m_patrol_and_disinfect', nurse_, room_, spotrobot_, uvdrobot_)
    ]

methods.declare_task_methods('m_full_mission', [full_mission])
