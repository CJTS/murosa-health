import re
from collections import defaultdict

import re
from collections import defaultdict


def extract_agent_name(action):
    """
    Extract the action name and parameters from an instantiated action.

    Args:
        action (str):
            Action string in the format:
            'action_name(param1,param2,...)'.

    Returns:
        tuple[str | None, list[str]]:
            A tuple containing:
            - Action name.
            - List of parameters.

            Returns (None, []) if parsing fails.
    """
    match = re.match(r'(\w+)\((.*?)\)', action)

    if match:
        action_name = match.group(1)
        params = [param.strip() for param in match.group(2).split(',')]
        return action_name, params

    return None, []


def get_ordered_instantiated_variables(actions):
    """
    Build an ordered list of instantiated parameters found in the plan.

    The order is determined by the first appearance of each parameter
    across all actions. This ordering is later used to map instantiated
    objects to mission variables.

    Args:
        actions (list[str]):
            List of instantiated actions.

    Returns:
        list[str]:
            Ordered list of unique instantiated parameters.
    """
    instantiated_variables = []

    for action in actions:
        _, params = extract_agent_name(action)

        for param in params:
            # Preserve first appearance order while avoiding duplicates.
            if param not in instantiated_variables:
                instantiated_variables.append(param)

    return instantiated_variables


def map_instantiated_to_variables(variables, ordered_instantiated_variables):
    """
    Create a mapping between instantiated objects and mission variables.

    Args:
        variables (list[str]):
            Variables extracted from the mission context.

        ordered_instantiated_variables (list[str]):
            Ordered instantiated parameters extracted from the plan.

    Returns:
        dict[str, str]:
            Mapping from instantiated objects to mission variables.
    """
    mapped_params = {}

    for index, param in enumerate(ordered_instantiated_variables):
        mapped_params[param] = variables[index]

    return mapped_params


def map_params_to_variables(params, variables_map):
    """
    Replace instantiated parameters with their corresponding variables.

    Args:
        params (list[str]):
            Instantiated action parameters.

        variables_map (dict[str, str]):
            Mapping from instantiated objects to mission variables.

    Returns:
        list[str]:
            Parameters represented using mission variables.
    """
    return [variables_map[param] for param in params]


def is_agent_first_action(agent, action, actions):
    """
    Determine whether an action is the first action assigned to an agent.

    Args:
        agent (str):
            Agent identifier.

        action (str):
            Action being evaluated.

        actions (list[str]):
            Complete mission plan.

    Returns:
        bool:
            True if the action is the first occurrence involving the
            specified agent, otherwise False.
    """
    agent_actions = []

    for current_action in actions:
        if agent in current_action:
            agent_actions.append(current_action)

    return action == agent_actions[0]


def build_agent_action_lists(agents, actions):
    """
    Group actions by participating agent.

    This transforms a global mission plan into independent execution
    sequences, one for each agent.

    Args:
        agents (list[str]):
            List of agent identifiers.

        actions (list[str]):
            Complete mission plan.

    Returns:
        dict[str, list[str]]:
            Dictionary mapping each agent to its ordered action sequence.
    """
    agent_actions = defaultdict(list)

    for action in actions:
        _, params = extract_agent_name(action)

        agents_in_action = set(params) & set(agents)

        for agent in agents_in_action:
            agent_actions[agent].append(action)

    return agent_actions


def generate_bdi(agents, actions, context, variables):
    """
    Generate BDI plans from a multi-agent mission plan.

    The generated plans follow an agent-centric execution model in which
    each agent maintains its own milestone chain. This allows independent
    agents to execute concurrently while preserving the ordering of
    actions assigned to the same agent.

    Args:
        agents (list[str]):
            Available agents in the mission.

        actions (list[str]):
            Instantiated mission plan.

        context (str):
            Mission start belief.

        variables (list[str]):
            Variables used by the mission context.

    Returns:
        defaultdict[list]:
            Mapping between agents and their generated BDI plans.
    """

    # Build an ordered mapping between instantiated objects and
    # context variables.
    ordered_instantiated_variables = get_ordered_instantiated_variables(actions)

    variables_map = map_instantiated_to_variables(
        variables,
        ordered_instantiated_variables
    )

    bdies = defaultdict(list)

    # Split the global mission plan into per-agent execution sequences.
    agent_actions = build_agent_action_lists(agents, actions)

    # Generate an independent execution chain for each agent.
    for agent, sequence in agent_actions.items():
        for local_idx in range(len(sequence) - 1):

            current_action = sequence[local_idx]
            next_action = sequence[local_idx + 1]

            action1, params1 = extract_agent_name(current_action)
            action2, params2 = extract_agent_name(next_action)

            # Resolve instantiated parameters to mission variables.
            mapped_params1 = map_params_to_variables(
                params1,
                variables_map
            )

            mapped_params2 = map_params_to_variables(
                params2,
                variables_map
            )

            action1_with_params = (
                f"{action1}({', '.join(mapped_params1)})"
            )

            action2_with_params = (
                f"{action2}({', '.join(mapped_params2)})"
            )

            # The first action is triggered directly by the mission
            # context. Subsequent actions are enabled by milestones.
            pre_condition = (
                context
                if is_agent_first_action(agent,
                                         current_action,
                                         actions)
                else f"{context} & milestone_{agent}_{local_idx}"
            )

            # Create the initial trigger rule for the first action.
            if is_agent_first_action(agent, current_action, actions):
                bdies[agent].append(
                    f"+{context}:\n\t{context} <-\n\t"
                    f"!{action1_with_params}.\n"
                )

            # Add the action execution rule.
            bdies[agent].append(
                f"+!{action1_with_params}:\n\t"
                f"{pre_condition} & not low_battery <-\n\t"
                f"{action1_with_params}.\n"
            )

            escape = "\n\t"
            # On success, activate the next milestone and trigger the
            # next action in the local execution chain.
            bdies[agent].append(
                f"+success_{action1_with_params}:\n\t"
                f"{context}"
                f"{(f' & milestone_{agent}_{local_idx}' if local_idx > 0 else '')}"
                f" <-\n\t"
                f"{(f'-milestone_{agent}_{local_idx};{escape}' if local_idx > 0 else '')}"
                f"+milestone_{agent}_{local_idx + 1};\n\t"
                f"!{action2_with_params}.\n"
            )

        # ----------------------------------------------------------
        # Process the final action separately because it terminates
        # the mission instead of triggering another action.
        # ----------------------------------------------------------

        local_idx += 1

        current_action = sequence[local_idx]

        action1, params1 = extract_agent_name(current_action)

        mapped_params1 = map_params_to_variables(
            params1,
            variables_map
        )

        action1_with_params = (
            f"{action1}({', '.join(mapped_params1)})"
        )

        # Generate the execution rule for the final action.
        bdies[agent].append(
            f"+!{action1_with_params}:\n\t"
            f"{context} & not low_battery"
            f" & milestone_{agent}_{local_idx}"
            f"<-\n\t{action1_with_params}.\n"
        )

        # Generate the success rule that completes the mission.
        bdies[agent].append(
            f"+success_{action1_with_params}:\n\t"
            f"{context} "
            f"& milestone_{agent}_{local_idx} "
            f"<-\n\t"
            f"-milestone_{agent}_{local_idx};\n\t"
            f"-{context};\n\t"
            f"-{action1_with_params};\n\t"
            f"end.\n"
        )

        # ==========================================================
        # Mission stop rules
        # ==========================================================

        stop_lines = []

        stop_lines.append(f"+stop: {context} <-")

        # Remove all milestones associated with this agent.
        for milestone in range(1, len(sequence)):
            stop_lines.append(
                f"    -milestone_{agent}_{milestone};"
            )

        # Remove the success belief of the final action.
        last_action, last_params = extract_agent_name(sequence[-1])

        mapped_last_params = map_params_to_variables(
            last_params,
            variables_map
        )

        last_action_with_params = (
            f"{last_action}({', '.join(mapped_last_params)})"
        )

        stop_lines.append(
            f"    -success_{last_action_with_params};"
        )

        # Remove mission context beliefs.
        stop_lines.append(f"    -{context};")
        stop_lines.append("    stop;\n")
        stop_lines.append("    -stop.\n")

        bdies[agent].insert(
            0,
            "\n".join(stop_lines)
        )

        # ==========================================================
        # Battery recovery plans
        # ==========================================================

        bdies[agent].append(
            """+low_battery_failure(Task): true <-
    .print("Charging");
    +after_charging(Task);
    +low_battery;
    a_charge.
"""
        )

        bdies[agent].append(
            """+success_a_charge: low_battery & after_charging(Task) <-
    .print("Finished charging");
    -after_charging(Task);
    -low_battery;
    !Task."""
        )

    return bdies

# Example usage
context = "start(Nurse, NurseRoom, SmallDeliveryRobot, SmallStorage, SmallResource, LargeDeliveryRobot, LargeStorage, LargeResource, Collector, ArmRoom, Arm, SpotRobot, UvdRobot)"
variables = [
    "Nurse",
    "NurseRoom",
    "SmallDeliveryRobot",
    "SmallStorage",
    "SmallResource",
    "LargeDeliveryRobot",
    "LargeStorage",
    "LargeResource",
    "Collector",
    "ArmRoom",
    "Arm",
    "SpotRobot",
    "UvdRobot"
]
agents = ['nurse1', 'small_delivery_robot1', 'large_delivery_robot1', 'collector1', 'arm1', 'spot1', 'uvd1']
actions = [
    'a_navto(nurse1,room1)',
    'a_navto(small_delivery_robot1,stor1)',
    'a_request_resource(small_delivery_robot1,stor1,resource1)',
    'a_pick_resource(small_delivery_robot1,stor1,resource1)',
    'a_navto(small_delivery_robot1,room1)',
    'a_deliver_resource(small_delivery_robot1,room1)',
    'a_navto(large_delivery_robot1,stor2)',
    'a_request_resource(large_delivery_robot1,stor2,resource2)',
    'a_pick_resource(large_delivery_robot1,stor2,resource2)',
    'a_navto(large_delivery_robot1,room1)',
    'a_deliver_resource(large_delivery_robot1,room1)',
    'a_collect_sample(nurse1,room1)',
    'a_navto(collector1,room1)',
    'a_approach_nurse(collector1,nurse1)',
    'a_authenticate_nurse(collector1,nurse1)',
    'a_open_drawer(collector1)',
    'a_deposit(nurse1,collector1)',
    'a_close_drawer(collector1)',
    'a_navto(collector1,ds)',
    'a_approach_arm(collector1,arm1)',
    'a_open_drawer(collector1)',
    'a_pick_up_sample(arm1,collector1)',
    'a_close_drawer(collector1)',
    'a_navto(spot1,room1)',
    'a_approach_nurse(spot1,nurse1)',
    'a_authenticate_nurse(spot1,nurse1)',
    'a_authorize_patrol(spot1,nurse1)',
    'a_patrol_room(spot1,room1)',
    'a_authorize_disinfect(uvd1,spot1)',
    'a_navto(uvd1,room1)',
    'a_disinfect_room(uvd1,room1)'
]

bdies = generate_bdi(agents, actions, context, variables)
for agente, regras in bdies.items():
    print(f"\n/* {agente} */")
    for regra in regras:
        print(regra)

