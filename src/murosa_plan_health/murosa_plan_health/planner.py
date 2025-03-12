from murosa_plan_health.hospital.domain.hospital_mod_methods import methods
from murosa_plan_health.hospital.domain.hospital_mod_actions import actions
from murosa_plan_health.hospital.problem.hospital_mod_problem import init_state
from murosa_plan_health.ipyhop import IPyHOP, planar_plot

print("Methods")
print(methods)
print("Actions")
print(actions)
print("Init state")
print(init_state)
planner = IPyHOP(methods, actions)
plan = planner.plan(init_state, [('m_pickup_and_deliver_sample', 'robot1', 'nurse1', 'arm1')], verbose=1)
# plan = planner.plan(init_state, [(goal)], verbose=1)

print(plan)
graph = planner.sol_tree
planar_plot(graph, root_node=0)

print('Plan: ')
for action in plan:
    print('\t', action)
 