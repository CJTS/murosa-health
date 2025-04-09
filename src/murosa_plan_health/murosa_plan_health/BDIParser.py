import re
from collections import defaultdict

def extract_agent_name(action):
    match = re.match(r'(\w+)\((.*?)\)', action)
    if match:
        action = match.group(1)
        params = [ag.strip() for ag in match.group(2).split(',')]
        return action, params
    return None, []

def agents_as_bdi_variabel(agents):
    pattern = r'[0-9]'
    return  [re.sub(pattern, '', ag.strip()).capitalize() for ag in agents]

def generate_bdi(agents, actions):
    bdies = defaultdict(list)
    i = 0
    while i < len(actions) - 1:
        current_action = actions[i]
        next_action = actions[i + 1]

        action1, params1 = extract_agent_name(current_action)
        action2, params2 = extract_agent_name(next_action)

        agents1 = set(params1) & set(agents)
        agents2 = set(params2) & set(agents)
        other_agents = list(set(agents2) - set(agents1))

        action1_with_params = f"{action1}({', '.join(agents_as_bdi_variabel(params1))})"
        action2_with_params = f"{action2}({', '.join(agents_as_bdi_variabel(params2))})"

        # Verifica quantas vezes o agente aparece nas ações
        count = defaultdict(int)
        for ag in list(agents1) + list(agents2):
            count[ag] += 1

        # Para cada agente da ação atual
        for agent1 in agents1:
            # Cria o plano da execução da ação atual
            bdies[agent1].append(f"+!{action1_with_params}: true <- {action1_with_params}.")
            # Se existem outros agentes
            if len(other_agents) > 0:
                # Cria o sends para todos os outros agentes
                success_plan = '; '.join([f".send({agent2}, tell, trigger_{action2_with_params})" for agent2 in other_agents])
                # Verifica se o agente atual esta na lista de proximos agentes
                if(agent1 in agents2):
                    # Se sim, adiciona a chamada para a proxima ação no plano de sucesso junto dos sends
                    bdies[agent1].append(f"+success_{action1_with_params}: true <- {success_plan}; +!{action2_with_params}.")
                else:
                    # Se não, adiciona somente os sends para os outros agentes executarem a proxima ação
                    bdies[agent1].append(f"+success_{action1_with_params}: true <- {success_plan}.")

                # para cada outro agente
                for agent2 in other_agents:
                    # crie o trigger para começar a proxima ação
                    bdies[agent2].append(f"+trigger_{action2_with_params}: true <- !{action2_with_params}.")
            # Se não existem outros agentes
            else:
                # Verifica se o agente atual esta na lista de proximos agentes
                if(agent1 in agents2):
                    # Se sim, adiciona a chamada para a proxima ação no plano de sucesso
                    bdies[agent1].append(f"+success_{action1_with_params}: true <- +!{action2_with_params}.")
                # else:
                    # bdies[agent1].append(f"+success_{action1_with_params}: true <- .")
        i += 1

    return bdies

# agents = ["nurse1", "arm2", "robot1"]

# # Exemplo de uso
# actions = [
#    "a_open_door(nurse1,room1)",
#    "a_navto(robot1,room1)",
#    "a_approach_nurse(robot1)",
#    "a_authenticate_nurse(robot1,nurse1)",
#    "a_open_drawer(robot1)",
#    "a_deposit(nurse1,robot1)",
#    "a_close_drawer(robot1)",
#    "a_navto(robot1,room4)",
#    "a_approach_arm(robot1,arm2)",
#    "a_open_drawer(robot1)",
#    "a_pick_up_sample(arm2,robot1)"
# ]

# bdis = generate_bdi(agents, actions)
# for agente, regras in bdis.items():
#     print(f"\n/* {agente} */")
#     for regra in regras:
#         print(regra)
