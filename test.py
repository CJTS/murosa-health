import inspect as i
import re

def transform_in_bdi(action_ipythop):
    print(action_ipythop)

def a_open_door(state, nurse_, room_):
    state.doors[room_] = True
    return state

print(transform_in_bdi(i.getsource(a_open_door)))


def generate_bdi_plans(function_def: str):
    lines = function_def.strip().split("\n")
    header = lines[0]
    conditions = []
    effects = []

    # Extract function name and parameters
    match = re.match(r"def (\w+)\((.*?)\):", header)
    if not match:
        raise ValueError("Invalid function definition")
    func_name = match.group(1)
    params = [p.strip() for p in match.group(2).split(",")]
    success_func_name = f"success_{func_name}"

    # Extract conditions and effects
    for line in lines[1:]:
        line = line.strip()
        if line.startswith("if"):
            condition_part = line[3:].strip().rstrip(":")
            conditions = condition_part.split(" and ")
        elif "= True" in line or "= False" in line:
            belief_match = re.findall(r"state\.(\w+)\[(\w+)\]", line)
            if belief_match:
                for belief, var in belief_match:
                    if "= True" in line:
                        effects.append(f"+{belief}({var}).")
                    else:
                        effects.append(f"-{belief}({var}).")

    # Convert conditions to belief syntax
    preconditions = []
    for cond in conditions:
        match = re.findall(r"state\.(\w+)\[(\w+)\]", cond)
        if match:
            for belief, var in match:
                preconditions.append(f"{belief}({var}).")

    # Generate plans
    param_str = ", ".join(params)
    plan1 = f"""
+!{func_name}({param_str}): {' & '.join(preconditions)} <-
    .print(\"Executing {func_name}\");
    {func_name}({param_str}).
    """.strip()

    print(effects)

    plan2 = f"""
+{success_func_name}({param_str}): {' & '.join(preconditions)} <-
    .print(\"{func_name} completed\");
    """.strip()

    return plan1 + "\n\n" + plan2

def parse_pddl_action(pddl_action: str) -> str:
    match = re.search(r'\(:action (\w+)', pddl_action)
    if not match:
        raise ValueError("Invalid PDDL action format")
    action_name = match.group(1)

    parameters_match = re.search(r':parameters \((.*?)\)', pddl_action, re.DOTALL)
    if not parameters_match:
        raise ValueError("Invalid PDDL parameters format")
    parameters = parameters_match.group(1)
    param_list = re.findall(r'\?(\w+) - \w+', parameters)
    param_str = ", ".join(param_list) + "_"

    preconditions_match = re.search(r':precondition \(and(.*?)\)', pddl_action, re.DOTALL)
    if not preconditions_match:
        raise ValueError("Invalid PDDL precondition format")
    preconditions = preconditions_match.group(1)
    precondition_facts = re.findall(r'\((\w+) (.*?)\)', preconditions)
    precondition_bdi = " & ".join([f"{fact}({', '.join(args.split())})" for fact, args in precondition_facts])

    effects_match = re.search(r':effect \(and(.*?)\)', pddl_action, re.DOTALL)
    if not effects_match:
        raise ValueError("Invalid PDDL effect format")
    effects = effects_match.group(1)
    effect_facts = re.findall(r'\((\w+) (.*?)\)', effects)
    effect_bdi = "\n    ".join([f"+{fact}({', '.join(args.split())});" for fact, args in effect_facts])

    bdi_action = f"+!a_{action_name}({param_str}): {precondition_bdi} <-\n"
    bdi_action += f'    .print("Approaching");\n'
    bdi_action += f'    a_{action_name}({param_str}).\n\n'
    
    bdi_success = f"+success_a_{action_name}({param_str}): {precondition_bdi} & {' & '.join([f'not {fact}({', '.join(args.split())})' for fact, args in effect_facts])} <-\n"
    bdi_success += f'    .print("Approaching complete");\n'
    bdi_success += f'    {effect_bdi}\n'
    bdi_success += f'    .send({param_list[1]}_,tell,robot_arrived({param_list[0]}_, Room));\n'
    bdi_success += f'    !a_authenticate_nurse({param_str}).\n'
    
    return bdi_action + bdi_success

# Example usage
function_description = """
def a_deposit(state, nurse_, robot_):
    if state.loc[nurse_] == state.loc[robot_] and state.sample[nurse_] == True:
        state.sample[nurse_] = False
        state.sample[robot_] = True
        return state
"""

print(generate_bdi_plans(function_description))

pddl_example = """
(:action approach_nurse
    :parameters (?robot - robot ?nurse - nurse ?room - room)
    :precondition (and
        (robot_at ?robot ?room)
        (nurse_at ?nurse ?room)
        (not (robot_near_nurse ?robot ?nurse))
    )
    :effect (and
        (robot_near_nurse ?robot ?nurse)
    )
)
"""

bdi_output = parse_pddl_action(pddl_example)
print(bdi_output)
