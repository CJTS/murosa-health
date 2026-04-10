#!/usr/bin/env python
"""
File Description: Hospital methods file. All the methods for Hospital planning domain are defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from coordinator.planner.health.domain.common_method import methods

# ******************************************        Method Definitions      ****************************************** #
def deliver_mission_empty(state, robot_, mission_, loc_):
    if not state.requested[mission_]:
        return []

def deliver_mission_non_empty(state, robot_, mission_, loc_):
    if not state.requested[mission_]:
        return False

    task_ = state.requested[mission_][0]
    resource_, storage_ = task_

    return [
        ('m_deliver_resource_task', robot_, resource_, storage_, loc_),
        ('m_remove_completed_task', mission_, task_),
        ('m_deliver_mission', robot_, mission_, loc_)
    ]

methods.declare_task_methods('m_deliver_mission', [
    deliver_mission_empty,
    deliver_mission_non_empty
])

def remove_completed_task(state, mission_, task_):
    state.requested[mission_].remove(task_)
    return []
methods.declare_task_methods('m_remove_completed_task', [remove_completed_task])

def deliver_resource_task(state, robot_, resource_, storage_, loc_):
    return [
        ('m_collect_resource', robot_, resource_, storage_),
        ('m_execute_delivery', robot_, loc_)
    ]
methods.declare_task_methods('m_deliver_resource_task', [deliver_resource_task])

def collect_resource_normal(state, robot_, resource_, storage_):
    if state.low_battery[robot_]:
        return False

    loc_ = storage_

    return [
        ('a_navto', robot_, loc_),
        ('a_request_resource', robot_, storage_, resource_),
        ('a_pick_resource', robot_, storage_, resource_)
    ]

methods.declare_task_methods(
    'm_collect_resource',
    [collect_resource_normal]
)

def deliver_normal(state, robot_, loc_):
    if state.low_battery[robot_]:
        return False

    return [
        ('a_navto', robot_, loc_),
        ('a_deliver_resource', robot_, loc_)
    ]

# def deliver_low_battery(state, robot_, task_):
#     if not state.low_battery[robot_]:
#         return False

#     resource_, _ = state.requested[task_]

#     checkpoint_ = list(state.checkpoint_loc.keys())[0]
#     loc_ = state.checkpoint_loc[checkpoint_]

#     return [
#         ('a_navto', robot_, loc_),
#         ('a_drop_checkpoint', robot_, checkpoint_),
#         ('m_recover_from_checkpoint', resource_, task_)
#     ]

methods.declare_task_methods(
    'm_execute_delivery',
    [deliver_normal]
)

def recover_from_checkpoint(state, resource_, task_):
    checkpoint_ = state.at_checkpoint.get(resource_)

    if not checkpoint_:
        return False

    loc_ = state.checkpoint_loc[checkpoint_]

    # find available robot
    for r in state.low_battery:
        if not state.low_battery[r]:
            return [
                ('a_navto', r, loc_),
                ('a_pick_checkpoint', r, resource_, checkpoint_),
                ('m_execute_delivery', r, task_)
            ]

methods.declare_task_methods(
    'm_recover_from_checkpoint',
    [recover_from_checkpoint]
)

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Methods isn't implemented.")
