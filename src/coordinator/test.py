from coordinator.planner.health.domain.methods import methods
from coordinator.planner.health.domain.actions import actions
# from coordinator.planner.health.problem.problem import init_state
from coordinator.planner.ipyhop import IPyHOP, planar_plot
from coordinator.planner.ipyhop import State

print(methods)
print(actions)
planner = IPyHOP(methods, actions)

init_state = State('init_state')
init_state.loc = {'nurse1': 'int2', 'nurse2': 'nr', 'nurse3': 'nr', 'nurse4': 'nr', 'uvd1': 'ds', 'spot1': 'int2', 'uvd2': 'ds', 'spot2': 'ds', 'collector1': 'int2', 'collector2': 'ds', 'arm1': 'lab', 'small_delivery_robot1': 'ds', 'small_delivery_robot2': 'stor2', 'large_delivery_robot1': 'stor3', 'large_delivery_robot2': 'ds'}
init_state.doors = {'room1': False, 'room2': False, 'room3': False, 'room4': False, 'room5': False, 'room6': False, 'icu': False, 'lab': True, 'int1': True, 'int2': True, 'int3': True, 'int4': True, 'int5': True, 'int6': True, 'int7': True, 'int8': True, 'int9': True, 'int10': True, 'nr': True, 'ds': True, 'stor1': True, 'stor2': True, 'stor3': True, 'stor4': True}
init_state.sample = {'room1': False, 'room2': False, 'room3': False, 'room4': False, 'room5': False, 'room6': False, 'icu': False, 'nurse1': False, 'nurse2': False, 'nurse3': False, 'nurse4': False, 'uvdrobot1': False, 'spotrobot1': False, 'uvdrobot2': False, 'spotrobot2': False, 'collector1': False, 'collector2': False, 'arm1': False}
init_state.cleaned = {'room1': False, 'room2': False, 'room3': False, 'room4': False, 'room5': False, 'room6': False, 'icu': False}
init_state.disinfected = {'room1': True, 'room2': True, 'room3': True, 'room4': True, 'room5': True, 'room6': True, 'icu': True}
init_state.low_battery = {'uvd1': False, 'spot1': False, 'uvd2': False, 'spot2': False, 'collector1': False, 'collector2': False, 'arm1': False}
init_state.resource_at = {'resource1': 'stor1', 'resource2': 'stor2', 'resource3': 'stor4', 'resource4': 'stor3'}
init_state.carrying = {'collector1': None, 'collector2': None, 'collector3': None}
init_state.requested = {}
init_state.samples = {'room1': False, 'room2': False, 'room3': False, 'room4': False, 'room5': False, 'room6': False, 'icu': False}


plan = planner.plan(init_state, [(
    'm_full_mission',
    'small_delivery_robot1',
    'stor1',
    'resource1',
    'large_delivery_robot1',
    'stor2',
    'resource2',
    'room1',
    'nurse1',
    'collector1',
    'arm1',
    'spot1',
    'uvd1'
)], verbose=1)

for action in plan:
    print(','.join(action))

graph = planner.sol_tree

planar_plot(graph)