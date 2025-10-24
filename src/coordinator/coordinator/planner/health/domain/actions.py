#!/usr/bin/env python
"""
File Description: Hospital actions file. All the actions for Hospital planning domain are defined here.
"""
# ******************************************    Libraries to be imported    ****************************************** #
from coordinator.planner.ipyhop import Actions

# ******************************************        Action Definitions      ****************************************** #
actions = Actions()

# robot navigate to a location
def a_navto(state, robot_, loc_):
    state.loc[robot_] = loc_
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
    a_collect_sample
])

# ******************************************    Demo / Test Routine         ****************************************** #
if __name__ == '__main__':
    raise NotImplementedError(
        "Test run / Demo routine for Hospital Mod Commands isn't implemented.")
