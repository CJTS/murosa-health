from coordinator.planner.health.domain.methods import methods
from coordinator.planner.health.domain.actions import actions
from coordinator.planner.health.problem.problem import init_state
from coordinator.planner.ipyhop import IPyHOP, planar_plot

goal = 'm_deliver_resource_task'
planner = IPyHOP(methods, actions)
plan = planner.plan(init_state, [(
    goal, 'collector1', 'stor1', 'resource1', 'collector2', 'stor2', 'resource2', 'room1'
)], verbose=1)

for action in plan:
    print(','.join(action))

graph = planner.sol_tree

planar_plot(graph, root_node=0)