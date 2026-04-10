#!/usr/bin/env python
"""
File Description: Hospital actions file. All the actions for Hospital planning domain are defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from coordinator.planner.ipyhop import Actions

# ******************************************        Action Definitions      ****************************************** #
actions = Actions()

# robot navigate to a location
def a_navto(state, agent_, loc_):
    state.loc[agent_] = loc_
    return state

# robot opens to a location
def a_open_door(state, nurse_, room_):
    state.doors[room_] = True
    return state

# robot aproaches a nurse
def a_approach_nurse(state, robot_, nurse_):
    return state

# robot authenticate the nurse
def a_authenticate_nurse(state, robot_, _nurse):
    return state

# robot opens drawer for nurse to deposit sample
def a_open_drawer(state, robot_):
    return state

# robot opens drawer after nurse deposits sample
def a_close_drawer(state, robot_):
    return state

# nurse deposits in drawer sample
def a_deposit(state, nurse_, robot_):
    if state.loc[nurse_] == state.loc[robot_] and state.sample[nurse_] == True:
        state.sample[nurse_] = False
        state.sample[robot_] = True
        return state

# robot aproaches an arm
def a_approach_arm(state, robot_, arm_):
    if state.loc[arm_] == state.loc[robot_]:
        return state

# arm picks up sample in robot's drawer
def a_pick_up_sample(state, arm_, robot_):
    if state.loc[arm_] == state.loc[robot_] and state.sample[robot_] == True:
        state.sample[robot_] = False
        state.sample[arm_] = True
        return state
    

# robot navigate to a location
def a_navto(state, robot_, loc_):
    state.loc[robot_] = loc_
    return state


def a_authorize_patrol(state, spotrobot_, nurse_):
    return state

def a_patrol_room(state, spotrobot_, room_):
    return state

def a_authorize_disinfect(state, uvdrobot_, spotrobot_):
    return state

def a_clean_room(state, nurse_, room_):
    state.cleaned[room_] = True
    return state

def a_disinfect_room(state, uvdrobot_, room_):
    state.disinfected[room_] = True
    return state

def a_charge(state, robot_,):
    state.low_battery[robot_] = False
    return state

def a_collect_sample(state, nurse_, room_):
    state.sample[room_] = False
    state.sample[nurse_] = True
    return state

def a_request_resource(state, robot_, storage_, resource_):
    if state.resource_at.get(resource_) == storage_:
        return state

def a_pick_resource(state, robot_, storage_, resource_):
    if state.loc[robot_] == storage_:
        state.carrying[robot_] = resource_
        del state.resource_at[resource_]
        return state

def a_deliver_resource(state, robot_, loc_):
    resource_ = state.carrying.get(robot_)

    if state.carrying.get(robot_) == resource_ and state.loc[robot_] == loc_:
        state.carrying[robot_] = None
        return state

# def a_drop_checkpoint(state, robot_, checkpoint_):
#     res = state.carrying.get(robot_)

#     if res and state.loc[robot_] == state.checkpoint_loc[checkpoint_]:
#         state.carrying[robot_] = None
#         state.at_checkpoint[res] = checkpoint_
#         return state

# def a_pick_checkpoint(state, robot_, resource_, checkpoint_):
#     if state.at_checkpoint.get(resource_) == checkpoint_:
#         state.carrying[robot_] = resource_
#         del state.at_checkpoint[resource_]
#         return state


actions.declare_actions([
    a_navto,
    a_open_door,
    a_approach_nurse,
    a_authenticate_nurse,
    a_open_drawer,
    a_close_drawer,
    a_deposit,
    a_approach_arm,
    a_pick_up_sample,
    a_authorize_patrol,
    a_patrol_room,
    a_clean_room,
    a_disinfect_room,
    a_authorize_disinfect,
    a_charge,
    a_collect_sample,
    a_request_resource,
    a_pick_resource,
    a_deliver_resource,
    # a_drop_checkpoint,
    # a_pick_checkpoint,
])

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Commands isn't implemented.")
