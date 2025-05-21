from murosa_plan.ipyhop import IPyHOP, planar_plot
from murosa_plan.disinfect.problem.disinfect_problem import init_state
from murosa_plan.disinfect.domain.disinfect_actions import actions
from murosa_plan.disinfect.domain.disinfect_methods import methods

# Criar o planejador
planner = IPyHOP(methods, actions)

# Definir a tarefa inicial
tasks = [('m_patrol_and_disinfect', 'spotrobot', 'uvdrobot', 'nurse1')]

# Rodar o plano
plan = planner.plan(init_state, tasks, verbose=1)

# Mostrar o plano
print("Plano gerado:")
print(plan)