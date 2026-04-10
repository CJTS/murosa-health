from coordinator.planner.health.domain.methods import methods
from coordinator.planner.health.domain.actions import actions
from coordinator.planner.health.problem.problem import init_state
from coordinator.planner.ipyhop import IPyHOP

goal = 'm_deliver_mission'
planner = IPyHOP(methods, actions)
init_state.requested = {
    'mission1': [('resource1', 'stor1'), ('resource2', 'stor2')]
}
plan = planner.plan(init_state, [(
    goal, 'collector1', 'mission1', 'room1'
)], verbose=1)

print(plan)

# for action in plan:
#     print(','.join(action))
