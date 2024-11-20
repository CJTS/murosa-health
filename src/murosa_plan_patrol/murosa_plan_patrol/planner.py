from patrol.domain.patrol_methods import methods
from patrol.domain.patrol_actions import actions
from patrol.problem.patrol_problem import init_state
from murosa_plan_patrol.ipyhop import IPyHOP, planar_plot

print("Methods")
print(methods)
print("Actions")
print(actions)
print("Init state")
print(init_state)
planner = IPyHOP(methods, actions)
plan = planner.plan(init_state, [('m_patrol_all', 'r2d2')], verbose=1)
# plan = planner.plan(init_state, [(goal)], verbose=1)

print(plan)
graph = planner.sol_tree
planar_plot(graph, root_node=1)

print('Plan: ')
for action in plan:
    print('\t', action)
 