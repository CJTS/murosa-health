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

def is_from_another(milestone_sources, i, agent1):
    for src in milestone_sources['milestone' + str(i)]:
        if(src != agent1):
            return f"[source({src})]"
   
    return ''

def generate_bdi(agents, actions, context, variables):
    # Generate variables array based on action parameters
    ordered_instanciated_variables = get_ordered_instanciated_variables(actions)
    variables_map = map_intaciated_to_variables(variables, ordered_instanciated_variables)
    milestone_sources = defaultdict(set)
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
                bdies[agent1].append(f"+!{action1_with_params}: not low_battery & {context} <- {action1_with_params}.")
            else:
                bdies[agent1].append(f"+!{action1_with_params}: not low_battery {(f'& milestone{str(i)}' if i > 0 else '')} <- {action1_with_params}.")
            # Se existem outros agentes
            if len(other_agents) > 0:
                # Cria o sends para todos os outros agentes
                success_plan = '; '.join([f".send({variables_map[agent2]}, tell, trigger_{action2_with_params})" for agent2 in other_agents])
                send_milestone_plan = '; '.join([f".send({variables_map[agent2]}, tell, milestone{str(i+1)})" for agent2 in other_agents])

                # Verifica se o agente atual esta na lista de proximos agentes
                if(agent1 in agents2):
                    # Se sim, adiciona a chamada para a proxima ação no plano de sucesso junto dos sends
                    if i == 0:
                        bdies[agent1].append(f"+success_{action1_with_params}: {context} {(f'& milestone{str(i)}' if i > 0 else '')} <- {(f'-milestone{str(i)}{is_from_another(milestone_sources, i, variables_map[agent1])};' if i > 0 else '')} +milestone{str(i+1)}; {send_milestone_plan}; {success_plan}; -initial_trigger_{action1_with_params}; !{action2_with_params}.")
                    else:
                        bdies[agent1].append(f"+success_{action1_with_params}: {context} {(f'& milestone{str(i)}' if i > 0 else '')} <- {(f'-milestone{str(i)}{is_from_another(milestone_sources, i, variables_map[agent1])};' if i > 0 else '')} +milestone{str(i+1)}; {send_milestone_plan}; {success_plan}; !{action2_with_params}.")
                else:
                    # Se não, adiciona somente os sends para os outros agentes executarem a proxima ação
                    bdies[agent1].append(f"+success_{action1_with_params}: {context} <- {send_milestone_plan}; {success_plan}.")

                # para cada outro agente
                for agent2 in other_agents:
                    # crie o trigger para começar a proxima ação
                    bdies[agent2].append(f"+trigger_{action2_with_params}: {context} <- !{action2_with_params}; -trigger_{action2_with_params}[source({variables_map[agent1]})].")
                    milestone_sources[f"milestone{str(i+1)}"].add(variables_map[agent1])
            # Se não existem outros agentes
            else:
                # Verifica se o agente atual esta na lista de proximos agentes
                if i > 0:
                    if(agent1 in agents2):
                        # Se sim, adiciona a chamada para a proxima ação no plano de sucesso
                        bdies[agent1].append(f"+success_{action1_with_params}: {context} {(f'& milestone{str(i)}' if i > 0 else '')} <- {(f'-milestone{str(i)}{is_from_another(milestone_sources, i, variables_map[agent1])};' if i > 0 else '')} +milestone{str(i+1)}; !{action2_with_params}.")
                    else:
                        bdies[agent1].append(f"+success_{action1_with_params}: {(f'milestone{str(i)}' if i > 0 else f'true')} <- {(f'-milestone{str(i)}{is_from_another(milestone_sources, i, variables_map[agent1])}.' if i > 0 else '')}")
                else:
                    if(agent1 in agents2):
                        # Se sim, adiciona a chamada para a proxima ação no plano de sucesso
                        bdies[agent1].append(f"+success_{action1_with_params}: {context} <- +milestone{str(i+1)}; -initial_trigger_{action1_with_params}; !{action2_with_params}.")

        i += 1
        
    # Por ultimo
    current_action = actions[i]
    action1, params1 = extract_agent_name(current_action)
    agents1 = set(params1) & set(agents)
    mapped_params1 = map_params_to_variables(params1, variables_map)
    action1_with_params = f"{action1}({', '.join(mapped_params1)})"; 

    for agent1 in agents1:
        bdies[agent1].append(f"+!{action1_with_params}: not low_battery & {(f'milestone{str(i)}' if i > 0 else '')} <- {action1_with_params}.")
        bdies[agent1].append(f"+success_{action1_with_params}: {(f'milestone{str(i)}' if i > 0 else '')} <- {(f'-milestone{str(i)}{is_from_another(milestone_sources, i, variables_map[agent1])};' if i > 0 else '')} -{context}; -{action1_with_params}; end.")

    max_milestone = len(actions)

    # Build param string for start(...) belief
    start_params = ', '.join(variables)
    start_belief = f"start({start_params})"

    # Verify all agents have an end in last rule
    for agent in bdies:
        last_rule = bdies[agent][-1] if bdies[agent] else ""
        if last_rule and last_rule.endswith(".") and not last_rule.endswith("; end."):
            action = re.match(r'^\+!([^:]+):', bdies[agent][-2])
            bdies[agent][-1] = last_rule[:-1] + f"; -{context}; -{action.group(1)}; end."

        # ================================
        #  ADD STOP / START MISSION RULES
        # ================================
        stop_lines = []
        stop_lines.append(f"+stop: {start_belief} <- ")
        # Remove triggers for each action
        for act in actions:
            act_name, params = extract_agent_name(act)
            mapped_params = map_params_to_variables(params, variables_map)
            act_with_params = f"{act_name}({', '.join(mapped_params)})"
            stop_lines.append(f"    -trigger_{act_with_params}[source({mapped_params[0]})];")
        # Remove milestones
        for m in range(1, max_milestone+1):
            stop_lines.append(f"    -milestone{m};")
            if f"milestone{m}" in milestone_sources:
                for src in milestone_sources[f"milestone{m}"]:
                    stop_lines.append(f"    -milestone{m}[source({src})];")
        # Remove last success of last action
        last_action, last_params = extract_agent_name(actions[-1])
        mapped_last_params = map_params_to_variables(last_params, variables_map)
        last_action_with_params = f"{last_action}({', '.join(mapped_last_params)})"
        stop_lines.append(f"    -success_{last_action_with_params};")
        # Remove start belief itself
        stop_lines.append(f"    -{start_belief};")
        stop_lines.append(f"    -stop.")
        bdies[agent].insert(0, "\n".join(stop_lines))

        # First action of the plan
        first_action_name, first_params = extract_agent_name(actions[0])
        mapped_first_params = map_params_to_variables(first_params, variables_map)
        first_action_with_params = f"{first_action_name}({', '.join(mapped_first_params)})"

        # If this agent is part of the first action, trigger it
        if agent in first_params:
            bdies[agent].insert(1, f"+{start_belief}: true <- +{start_belief}; !{first_action_with_params}.")
        else:
            bdies[agent].insert(1, f"+{start_belief}: true <- +{start_belief}; .")



    # ================================
    #  ADD CHARGE PLANS
    # ================================
        bdies[agent].append("""+low_battery_failure(Task): true <- .print("Charging"); +after_charging(Task); +low_battery; a_charge.""")
        bdies[agent].append("""+success_a_charge: low_battery & after_charging(Task) <- .print("Finished charging"); -after_charging(Task); -low_battery; !Task.""")

    return bdies

# Example usage
context = "start(SpotRobot, NurseRoom, Nurse, UvdRobot)"
variables = ["SpotRobot", "NurseRoom", "Nurse", "UvdRobot"]
agents = ["spotrobot1", "uvdrobot2", "nurse_disinfected1"]

actions = [
    "a_navto(spotrobot1,room1)",
    "a_open_door(spotrobot1,room1)",
    "a_approach_nurse(spotrobot1,nurse_disinfected1)",
    "a_authenticate_nurse(spotrobot1,nurse_disinfected1)",
    "a_clean_room(nurse_disinfected1,room1)",
    "a_authorize_patrol(spotrobot1,nurse_disinfected1)",
    "a_patrol_room(spotrobot1,room1)",
    "a_authorize_disinfect(uvdrobot2,spotrobot1)",
    "a_navto(uvdrobot2,room1)",
    "a_disinfect_room(uvdrobot2,room1)"
]

bdis = generate_bdi(agents, actions, context, variables)
for agente, regras in bdis.items():
    print(f"\n/* {agente} */")
    for regra in regras:
        print(regra)
