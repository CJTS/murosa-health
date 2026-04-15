#!/usr/bin/env python
"""
File Description: Hospital methods file. All the methods for Hospital planning domain are defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from coordinator.planner.health.domain.common_method import methods

# ******************************************        Method Definitions      ****************************************** #
def deliver_resource_task(state, small_box_delivery_robot_, small_box_storage_, small_resource_, large_box_delivery_robot_, large_box_storage_, large_resource_, loc_):
    return [
        ('m_deliver_small_resource_task', small_box_delivery_robot_, small_box_storage_, small_resource_, loc_),
        ('m_deliver_large_resource_task', large_box_delivery_robot_, large_box_storage_, large_resource_, loc_),
    ]
methods.declare_task_methods('m_deliver_resource_task', [deliver_resource_task])

def deliver_small_resource_task(state, small_box_delivery_robot_, small_box_storage_, small_resource_, loc_):
    return [
        ('m_collect_resource', small_box_delivery_robot_, small_box_storage_, small_resource_),
        ('m_execute_delivery', small_box_delivery_robot_, loc_),
    ]
methods.declare_task_methods('m_deliver_small_resource_task', [deliver_small_resource_task])

def deliver_large_resource_task(state, large_box_delivery_robot_, large_box_storage_, large_resource_, loc_):
    return [
        ('m_collect_resource', large_box_delivery_robot_, large_box_storage_, large_resource_),
        ('m_execute_delivery', large_box_delivery_robot_, loc_),
    ]
methods.declare_task_methods('m_deliver_large_resource_task', [deliver_large_resource_task])

def collect_resource(state, box_delivery_robot_, box_storage_, resource_):
    return [
        ('a_navto', box_delivery_robot_, box_storage_),
        ('a_request_resource', box_delivery_robot_, box_storage_, resource_),
        ('a_pick_resource', box_delivery_robot_, box_storage_, resource_)
    ]

def assemble_collect_resource(state, box_delivery_robot_, box_storage_, resource_):
    return [
        ('a_navto', box_delivery_robot_, box_storage_),
        ('a_assemble_resource', box_delivery_robot_, box_storage_, resource_),
        ('a_pick_resource', box_delivery_robot_, box_storage_, resource_)
    ]

methods.declare_task_methods(
    'm_collect_resource',
    [collect_resource, assemble_collect_resource]
)

def deliver(state, box_delivery_robot_, loc_):
    return [
        ('a_navto', box_delivery_robot_, loc_),
        ('a_deliver_resource', box_delivery_robot_, loc_),
    ]
methods.declare_task_methods(
    'm_execute_delivery',
    [deliver]
)

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Methods isn't implemented.")
