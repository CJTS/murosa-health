import re
from collections import defaultdict

def extract_agent_name(action):
    match = re.match(r'(\w+)\((.*?)\)', action)
    if match:
        action = match.group(1)
        params = [ag.strip() for ag in match.group(2).split(',')]
        return action, params
    return None, []

def get_ordered_instanciated_variables(actions):
    """Generate variables array based on the order of parameter appearances in actions"""
    instanciated_variables = []
    for action in actions:
        _, params = extract_agent_name(action)
        for param in params:
            # Remove numbers and capitalize
            if param not in instanciated_variables:
                instanciated_variables.append(param)
    
    return instanciated_variables

def map_intaciated_to_variables(variables, ordered_instanciated_variables):
    mapped_params = {}
    i = 0
    for param in ordered_instanciated_variables:
        mapped_params[param] = variables[i]
        i += 1
    return mapped_params

def map_params_to_variables(params, variables_map):
    mapped_params = []
    for param in params:
        mapped_params.append(variables_map[param])
    return mapped_params

def generate_bdi(agents, actions, context, variables):
    # Generate variables array based on action parameters
    ordered_instanciated_variables = get_ordered_instanciated_variables(actions)
    variables_map = map_intaciated_to_variables(variables, ordered_instanciated_variables)
    
    bdies = defaultdict(list)
    i = 0
    # Da primeira ate a penultima ação
    while i < len(actions) - 1:
        current_action = actions[i]
        next_action = actions[i + 1]

        action1, params1 = extract_agent_name(current_action)
        action2, params2 = extract_agent_name(next_action)

        agents1 = set(params1) & set(agents)
        agents2 = set(params2) & set(agents)
        other_agents = list(set(agents2) - set(agents1))

        # Map parameters to variables
        mapped_params1 = map_params_to_variables(params1, variables_map)
        mapped_params2 = map_params_to_variables(params2, variables_map)

        action1_with_params = f"{action1}({', '.join(mapped_params1)})"
        action2_with_params = f"{action2}({', '.join(mapped_params2)})"

        # Verifica quantas vezes o agente aparece nas ações
        count = defaultdict(int)
        for ag in list(agents1) + list(agents2):
            count[ag] += 1

        # Para cada agente da ação atual
        for agent1 in agents1:
            # Cria o plano da execução da ação atual
            if(i == 0):
                bdies[agent1].append(f"+initial_trigger_{action1_with_params}: {context} <- !{action1_with_params}.")
                bdies[agent1].append(f"+!{action1_with_params}: {context} <- {action1_with_params}.")
            else:
                bdies[agent1].append(f"+!{action1_with_params}: milestone{str(i - 1)} <- {action1_with_params}.")

            # Se existem outros agentes
            if len(other_agents) > 0:
                # Cria o sends para todos os outros agentes
                success_plan = '; '.join([f".send({variables_map[agent2]}, tell, trigger_{action2_with_params})" for agent2 in other_agents])
                send_milestone_plan = '; '.join([f".send({variables_map[agent2]}, tell, milestone{str(i)})" for agent2 in other_agents])
                # Verifica se o agente atual esta na lista de proximos agentes
                if(agent1 in agents2):
                    # Se sim, adiciona a chamada para a proxima ação no plano de sucesso junto dos sends
                    bdies[agent1].append(f"+success_{action1_with_params}: {context} & milestone{str(i - 1)} <- -milestone{str(i - 1)}; +milestone{str(i)}; {send_milestone_plan}; {success_plan}; !{action2_with_params}.")
                else:
                    # Se não, adiciona somente os sends para os outros agentes executarem a proxima ação
                    bdies[agent1].append(f"+success_{action1_with_params}: {context} <- {send_milestone_plan}; {success_plan}.")

                # para cada outro agente
                for agent2 in other_agents:
                    # crie o trigger para começar a proxima ação
                    bdies[agent2].append(f"+trigger_{action2_with_params}: {context} <- !{action2_with_params}.")
            # Se não existem outros agentes
            else:
                # Verifica se o agente atual esta na lista de proximos agentes
                if i > 0:
                    if(agent1 in agents2):
                        # Se sim, adiciona a chamada para a proxima ação no plano de sucesso
                        bdies[agent1].append(f"+success_{action1_with_params}: {context} & milestone{str(i - 1)} <- -milestone{str(i - 1)}; +milestone{str(i)}; !{action2_with_params}.")
                    else:
                        bdies[agent1].append(f"+success_{action1_with_params}: milestone{str(i - 1)} <- -milestone{str(i - 1)}.")
                else:
                    if(agent1 in agents2):
                        # Se sim, adiciona a chamada para a proxima ação no plano de sucesso
                        bdies[agent1].append(f"+success_{action1_with_params}: {context} <- +milestone{str(i)}; !{action2_with_params}.")

        i += 1
    # Por ultimo
    current_action = actions[i]
    action1, params1 = extract_agent_name(current_action)
    agents1 = set(params1) & set(agents)
    mapped_params1 = map_params_to_variables(params1, variables_map)
    action1_with_params = f"{action1}({', '.join(mapped_params1)})"; 

    for agent1 in agents1:
        bdies[agent1].append(f"+!{action1_with_params}: milestone{str(i - 1)} <- {action1_with_params}.")
        bdies[agent1].append(f"+success_{action1_with_params}: milestone{str(i - 1)} <- -milestone{str(i - 1)}; end.")

    return bdies

# # Example usage
# # context = "start(Nurse, LockedDoor, Robot, ArmRoom, Arm)"
# context = "start(Patrol, Base, Room1, Room2, Room3, Room4)"
# # variables = ["Nurse", "LockedDoor", "Robot", "ArmRoom", "Arm"]
# # variables = ["Robot", "NurseRoom", "Nurse", "ArmRoom", "Arm"]
# variables = ["Patrol", "Room1", "Base", "Room2", "Room3", "Room4"]
# agents = ["patrol1"]

# actions = [
# #    "a_open_door(nurse1,room1)",
# #    "a_navto(robot1,room1)",
# #    "a_approach_nurse(robot1, nurse1)",
# #    "a_authenticate_nurse(robot1,nurse1)",
# #    "a_open_drawer(robot1)",
# #    "a_deposit(nurse1,robot1)",
# #    "a_close_drawer(robot1)",
# #    "a_navto(robot1,room4)",
# #    "a_approach_arm(robot1,arm2)",
# #    "a_open_drawer(robot1)",
# #    "a_pick_up_sample(arm2,robot1)"
#     "move(patrol1, wp1)",
#     "move(patrol1, wp_control)",
#     "move(patrol1, wp2)",
#     "move(patrol1, wp_control)",
#     "move(patrol1, wp3)",
#     "move(patrol1, wp_control)",
#     "move(patrol1, wp4)",
#     "move(patrol1, wp_control)",
# ]

# bdis = generate_bdi(agents, actions, context, variables)
# for agente, regras in bdis.items():
#     print(f"\n/* {agente} */")
#     for regra in regras:
#         print(regra)
