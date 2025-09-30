import rclpy
import json
import time
from rclpy.node import Node
from interfaces.srv import Message, Action
from std_msgs.msg import String, Bool
from coordinator.helper import FIPAMessage, action_string_to_tuple, action_tuple_to_string
from coordinator.BDIParser import generate_bdi
from coordinator.FIPAPerformatives import FIPAPerformative
from enum import Enum
from typing import List

class MissionStatus(Enum):
    CREATED = 1
    ONGOING = 2
    PAUSED = 3
    ERROR = 4
    FINISHED = 5

class MissionRobot():
    def __init__(self, robot):
        self.robot = robot
        self.finished = False
    
    def __str__(self):
        return self.robot
    
class Team():
    def __init__(self, robots: MissionRobot):
        self.robots = robots

class Mission():
    def __init__(self, team: List[MissionRobot], context):
        self.team = team
        self.status = MissionStatus.CREATED
        self.context = context
        self.priority = -1
        self.mission_context = "start()"
        self.variables = []

class AgnosticCoordinator(Node):
    def __init__(self, Name):
        super().__init__('AgnosticCoordinator')
        # List of missions
        self.missions: List[Mission] = []
        # List of messages to be sent to the agents
        self.register_queue = []
        # List of missions that have errors
        self.missions_with_error: List[Mission] = []
        self.known_errors = []
        self.declare_parameter('replan', rclpy.Parameter.Type.BOOL)
        replan_param = self.get_parameter('replan').get_parameter_value().bool_value
        self.should_replan = replan_param

        # Coordinator server
        self._action_server = self.create_service(
            Message,
            'coordinator',
            self.execute_callback
        )
        self.jason_publisher = self.create_publisher(String, '/coordinator/jason/plan', 10)
        self.agent_publisher = self.create_publisher(String, '/coordinator/agent/plan', 10)

        # Subscriber para falar com os agents (Ação)
        self.subscription = self.create_subscription(
            String, '/agent/coordinator/action', self.listener_callback, 10
        )
        self.result_subscription = self.create_subscription(
            String, '/agent/coordinator/result', self.listener_callback, 10
        )

        self.environment_client = self.create_client(
            Action, 'environment_server'
        )
        while not self.environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('environment service not available, waiting again...')

        self.planner_communication_sync_client = self.create_client(
            Action, 'planner_communication_sync_server'
        )
        while not self.planner_communication_sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('planner sync service not available, waiting again...')

        # Subscriber para indicar fim da execução
        self.end_subscription = self.create_subscription(
            Bool, '/jason/shutdown_signal', self.shutdown_callback, 10
        )
        self.end_publisher = self.create_publisher(Bool, '/coordinator/shutdown_signal', 10)

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento, finalizando...")
            raise SystemExit
        
    def listener_callback(self, msg):
        decoded_msg = FIPAMessage.decode(msg.data)
        # self.get_logger().info('I heard: "%s"' % msg.data)

        if decoded_msg.content == 'Ready':
            self.set_agent_ready(decoded_msg)
        elif decoded_msg.content == 'Finished':
            robot_name = decoded_msg.sender
            mission = self.get_mission(robot_name)

            if mission == None:
                self.get_logger().info(f"No mission found for {robot_name}")
                return

            self.mark_agent_finished(mission, robot_name)
            self.free_agent(robot_name)
            if self.verify_mission_complete(mission):
                self.finish_mission(mission)

    def get_mission(self, agent: str):
        for mission in self.missions:
            for mission_robot in mission.team:
                if mission_robot.robot == agent:
                    return mission
        return None

    def mark_agent_finished(self, mission: Mission, agent: str):
        for mission_robot in mission.team:
            if mission_robot.robot == agent:
                mission_robot.finished = True
                self.get_logger().info(f"Marked {agent} as finished in mission.")
                break

    def free_agent(self, mission: Mission, agent: str):
        raise NotImplementedError("This method should be implemented by the subclass")

    def verify_mission_complete(self, mission: Mission):
        if all(mission_robot.finished for mission_robot in mission.team):
            mission.status = MissionStatus.FINISHED
            return True

        return False
    
    def finish_mission(self, finished_mission: Mission):
        self.get_logger().info("Mission Completed")
        self.get_logger().info(", ".join([str(robot) for robot in finished_mission.team]))
        self.missions.remove(finished_mission)
        # for agent in mission:
        #     msg = String()
        #     msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, agent, 'Jason', 'End|' + agent).encode()
        #     self.jason_publisher.publish(msg)
        self.get_logger().info(str(len(self.missions)))
        for mission in self.missions:
            self.get_logger().info(", ".join([str(robot) for robot in mission.team]))
        if(len(self.missions) == 0):
            self.end_simulation()

    def set_agent_ready(self, decoded_msg):
        raise NotImplementedError("This method should be implemented by the subclass")

    def execute_callback(self, request, response):
        decoded_msg = FIPAMessage.decode(request.content)
        # self.get_logger().info(f'Received: Performative={decoded_msg.performative}, Sender={decoded_msg.sender}, Receiver={decoded_msg.receiver}, Content={decoded_msg.content}')

        if decoded_msg.performative == FIPAPerformative.REQUEST.value:
            if decoded_msg.content == 'Register':
                response.response = self.register_agent(decoded_msg)
        elif decoded_msg.performative == FIPAPerformative.INFORM.value:
            if "ERROR" in decoded_msg.content:
                mission = [mission for mission in self.missions if decoded_msg.sender in mission]
                self.missions_with_error.append((mission[0], decoded_msg.content))
        return response
    
    def register_agent(self, decoded_msg):
        raise NotImplementedError("This method should be implemented by the subclass")

    def restart_mission(self, mission):
        raise NotImplementedError("This method should be implemented by the subclass")

    def start_mission(self) -> Mission:
        team = self.get_team()
        if(team == None):
            # self.get_logger().info("No team to start mission")
            return None

        start_context = self.get_start_context(team)

        if(start_context == None):
            # self.get_logger().info("Not possible to start mission")
            return None
        
        mission = self.create_mission(team, start_context)
    
        for agent in team:
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', agent.robot, 'Start|' + ','.join(start_context)).encode()
            self.agent_publisher.publish(msg)

        return mission
    
    def get_team(self) -> List[MissionRobot]:
        raise NotImplementedError("This method should be implemented by the subclass")

    def get_start_context(self, team):
        raise NotImplementedError("This method should be implemented by the subclass")
    
    def create_mission(self, team: List[MissionRobot], start_context):
        raise NotImplementedError("This method should be implemented by the subclass")

    def get_agent_class(self, agent):
        # TODO only works for agents from 1 to 9
        return agent[:-1]
    
    def send_monitor_state_request(self, action):
        self.update_state_request = Action.Request()
        self.update_state_request.action = action
        return self.environment_client.call_async(self.update_state_request)

    def check_env(self):
        future = self.send_monitor_state_request(','.join(('monitor',)))
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.state = json.loads(response.observation)
        self.update_planner_state(response.observation)
        self.verify_initial_trigger()

    def verify_initial_trigger(self):
        raise NotImplementedError("This method should be implemented by the subclass")
    
    def send_update_state_request(self, state):
        self.update_state_request = Action.Request()
        self.update_state_request.action = state
        return self.planner_communication_sync_client.call_async(self.update_state_request)

    def update_planner_state(self, state):
        future = self.send_update_state_request('|'.join(('update_state', state)))
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def register_agents(self):
        if(len(self.register_queue) > 0):
            agent = self.register_queue.pop()
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', 'Jason', 'Create|' + ','.join(agent)).encode()
            self.jason_publisher.publish(msg)

    def fix_plan(self, context):
        self.get_logger().info(",".join(context))
        team = self.get_team_from_context(context)

        self.get_logger().info('Creating plan for: %s ' % (
            ','.join(team)
        ))
        future = self.send_need_plan_request(','.join(team))
        rclpy.spin_until_future_complete(self, future)
        plan_response = future.result()
        self.get_logger().info('Plan received for: %s ' % (
            ','.join(team)
        ))
        self.get_logger().info(plan_response.observation)
        self.current_plan = plan_response.observation.split('/')
        formated_plan = []
        start = []
        for action in self.current_plan:
            splitted_action = action.split(',')
            formated_plan.append(splitted_action[0] + "(" + ','.join(splitted_action[1:len(splitted_action)])  + ")")
            params = splitted_action[1:len(splitted_action)]
            for param in params:
                if param not in start:
                    start.append(param)

        bdies = generate_bdi(team, formated_plan, self.mission_context, self.variables)
        for agent, rules in bdies.items():
            plans = [f"+!{self.mission_context}: true <- +{self.mission_context}."]
            for rule in rules:
                plans.append(rule)
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.INFORM.value, 'Coordinator', agent, 'Plan|' + '/'.join(plans)).encode()
            self.agent_publisher.publish(msg)

        start_msg = "initial_trigger_" + formated_plan[0] + "."

        for agent in team:
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', agent, 'Start|' + ','.join(start)).encode()
            self.agent_publisher.publish(msg)

        splitted_action = self.current_plan[0].split(',')
    
        for initial_agents in splitted_action[1:len(splitted_action)]:
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.INFORM.value, 'Coordinator', initial_agents, 'Belief|' + start_msg).encode()
            self.agent_publisher.publish(msg)

    def get_team_from_context(self, context):
        raise NotImplementedError("This method should be implemented by the subclass")

    def split_plans(self, team):
        tuples = map(action_string_to_tuple, self.current_plan)

        for agent in team:
            self.agents_actions[agent] = []

            for action in tuples:
                if agent in action:
                    self.agents_actions[agent].append(action)

    def send_plans_request(self, team):
        # NOT AGNOSTIC
        self.get_logger().info('Sending plans')
        for agent in team:
            send_plan_request_nurse = String()
            send_plan_request_nurse.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', agent, 'Plan|' + '/'.join(list(map(
                action_tuple_to_string, self.agents_actions[agent]
            )))).encode()
            self.agent_publisher.publish(send_plan_request_nurse)

        self.get_logger().info('Plans sent')

    def fix_missions(self):
        if len(self.missions_with_error) > 0:
                mission_error = self.missions_with_error.pop()
                mission = mission_error[0]
                error = mission_error[1]
                error_msg = error.split("|")
                error_desc = error_msg[1].split(",")
                if error_desc[0] in self.known_errors:
                    self.get_logger().info('Restarting')
                    time.sleep(1)
                    self.restart_mission(mission)
                else:
                    if self.should_replan:
                            self.get_logger().info('Error found')
                            self.fix_plan(mission)
                    else:
                        self.get_logger().info('Error found')
                        self.end_simulation()

    def end_simulation(self):
        self.get_logger().info('Ending simulation')
        msg = Bool()
        msg.data = True
        self.end_publisher.publish(msg)
        raise SystemExit

    def send_need_plan_request(self, team):
        self.action_request = Action.Request()
        self.action_request.action = 'need_plan|' + team
        return self.planner_communication_sync_client.call_async(self.action_request)
    
    def stop_low_priority_mission(self):
        low_priority_missions = [mission for mission in self.missions if mission.priority == 0]
        
        if len(low_priority_missions) == 0:
            return False
        
        low_mission = low_priority_missions[0]

        self.get_logger().info("Missions vefore stopping low priority mission:")
        self.get_logger().info(str(len(self.missions)))
        for mission in self.missions:
            self.get_logger().info(", ".join([str(robot) for robot in mission.team]))

        self.get_logger().info(f"Stopping low priority mission with team: {', '.join([str(robot) for robot in low_mission.team])}")

        for agent in low_mission.team:
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', 'Jason', 'Stop|' + agent.robot).encode()
            self.agent_publisher.publish(msg)
            self.free_agent(agent.robot)
        self.missions.remove(low_mission)

        self.get_logger().info("Missions after stopping low priority mission:")
        self.get_logger().info(str(len(self.missions)))
        for mission in self.missions:
            self.get_logger().info(", ".join([str(robot) for robot in mission.team]))


        return True

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            self.register_agents()
            self.fix_missions()
            self.check_env()
