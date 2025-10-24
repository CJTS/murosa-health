import rclpy
import json
import time
import re

from rclpy.node import Node
from std_msgs.msg import String, Bool
from interfaces.srv import Message, Action
from coordinator.helpers.helper import FIPAMessage, action_string_to_tuple, action_tuple_to_string
from coordinator.helpers.BDIParser import generate_bdi
from coordinator.helpers.FIPAPerformatives import FIPAPerformative
from enum import Enum
from typing import List

class MissionStatus(Enum):
    CREATED = 1
    WAITING_TEAM = 2
    RUNNING = 3
    ERROR = 5
    FINISHED = 6
    CANCELED = 7

class RobotStatus(Enum):
    CREATED = 1
    READY = 2
    OCCUPIED = 3

class MissionRobot():
    def __init__(self, robot):
        self.robot = robot
        self.finished = False
        self.trigger = None
        self.plan_version = 1
        self.current_bdi = {}
        self.status = RobotStatus.CREATED

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
        self.plan = []
        self.state = {}
        self.error = None

class AgnosticCoordinator(Node):
    def __init__(self, name):
        """
        1. Initialize coordinator variables, like missions, robots, and known errors.
        Initialize env variables
        Initialize ros2 communication
        """
        super().__init__('AgnosticCoordinator')
        self.missions: List[Mission] = []
        self.robots: List[MissionRobot] = []
        self.register_queue = []
        self.known_errors = []
        self.agents_actions = {}

        self.declare_parameter('replan', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('bdi', rclpy.Parameter.Type.BOOL)
        self.should_replan = self.get_parameter('replan').get_parameter_value().bool_value
        self.should_use_bdi = self.get_parameter('bdi').get_parameter_value().bool_value

        self.initialize_communication()

    def initialize_communication(self):
        # Coordinator servers
        self._action_server = self.create_service(
            Message,
            'coordinator',
            self.coordinator_server_callback
        )

        self.agent_publisher = self.create_publisher(String, '/coordinator/agent/plan', 10)
        self.agent_reset_publisher = self.create_publisher(String, '/coordinator/agent/reset', 10)
        self.end_publisher = self.create_publisher(Bool, '/coordinator/shutdown_signal', 10)

        if self.should_use_bdi:
            self.jason_publisher = self.create_publisher(String, '/coordinator/jason/plan', 10)

        # Coordinator subscribers
        self.subscription = self.create_subscription(
            String, '/agent/coordinator/action', self.listener_callback, 10
        )

        # Coordinator clients
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

    def coordinator_server_callback(self, request, response):
        """2. Receive request to coordinator, register agents, initial triggers and errors found"""
        decoded_msg = FIPAMessage.decode(request.content)

        if decoded_msg.performative == FIPAPerformative.REQUEST.value:
            if decoded_msg.content == 'Register':
                response.response = self.register_agent(decoded_msg)
        elif decoded_msg.performative == FIPAPerformative.INFORM.value:
            if "ERROR" in decoded_msg.content:
                self.get_logger().info('Error found')
                mission = self.get_mission(decoded_msg.sender)
                if mission is None:
                    self.get_logger().info(f"No mission found for {decoded_msg.sender}")
                    return response
                mission.status = MissionStatus.ERROR
                mission.error = decoded_msg.content
                self.stop_mission(mission)
            elif "InitialTrigger" in decoded_msg.content:
                self.initial_trigger(decoded_msg)
            elif "Ready" in decoded_msg.content:
                response.response = self.set_agent_ready(decoded_msg)

        return response

    def register_agent(self, decoded_msg):
        raise NotImplementedError("This method should be implemented by the subclass")

    def get_mission(self, agent: str):
        for mission in self.missions:
            for mission_robot in mission.team:
                if mission_robot.robot == agent:
                    return mission
        return None

    def stop_mission(self, mission: Mission):
        for agent in mission.team:
            if not agent.finished:
                msg = String()
                msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', agent.robot, 'Stop|' + agent.robot).encode()
                self.agent_publisher.publish(msg)

    def initial_trigger(self, msg):
        raise NotImplementedError("This method should be implemented by the subclass")

    def create_mission(self, mission_type, trigger):
        raise NotImplementedError("This method should be implemented by the subclass")

    def get_team(self, mission: Mission) -> List[MissionRobot]:
        team = []
        self.get_logger().info(str(mission))
        self.get_logger().info(str(mission.roles))
        for role in mission.roles:
            free_robot = next((robot for robot in self.robots if robot.status == RobotStatus.READY and robot.role == role), None)
            if free_robot is not None:
                team.append(free_robot)

        if(len(team) is not len(mission.roles)):
            return None

        for robot in team:
            robot.status = RobotStatus.OCCUPIED

        return team

    def get_start_context(self, mission_type, team: List[MissionRobot], room: str):
        raise NotImplementedError("This method should be implemented by the subclass")

    def set_agent_ready(self, decoded_msg):
        robot = next((robot for robot in self.robots if robot.robot == decoded_msg.sender), None)
        robot.status = RobotStatus.READY
        return 'success'

    def listener_callback(self, msg):
        """2. Receive messages from agents initial triggers and errors found"""
        decoded_msg = FIPAMessage.decode(msg.data)

        if decoded_msg.content == 'Finished':
            robot_name = decoded_msg.sender
            mission = self.get_mission(robot_name)

            if mission == None:
                self.get_logger().info(f"No mission found for {robot_name}")
                return

            self.mark_agent_finished(mission, robot_name)
            self.free_agent(robot_name)
            if self.verify_mission_complete(mission):
                self.finish_mission(mission)

    def analyze_missions(self):
        for mission in self.missions:
            if mission.status == MissionStatus.CREATED:
                self.start_mission(mission)
                mission.status = MissionStatus.RUNNING
            elif mission.status == MissionStatus.ERROR:
                self.fix_missions(mission)
                mission.status = MissionStatus.RUNNING
            elif mission.status == MissionStatus.WAITING_TEAM:
                # TODO check if a mission finished recently
                team = self.get_team(mission)
                if team == None:
                    continue
                self.get_logger().info("Team found to start mission")
                start_context = self.get_start_context(team, mission.trigger)
                # TODO NOT AGNOSTIC
                mission.team = team
                mission.context = start_context
                self.start_mission(mission)
                mission.status = MissionStatus.RUNNING

    def start_mission(self, mission: Mission) -> Mission:
        if self.should_use_bdi:
            for agent in mission.team:
                msg = String()
                msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', agent.robot, 'Start|' + ','.join(mission.context)).encode()
                self.agent_publisher.publish(msg)
        else:
            self.update_planner_state(json.dumps(self.state))
            future = self.send_need_plan_request(mission.type, ','.join(mission.context))
            rclpy.spin_until_future_complete(self, future)
            plan_response = future.result()
            self.get_logger().info('Plan received for: %s ' % (
                ','.join([str(robot) for robot in mission.team])
            ))
            if plan_response.observation == '':
                self.get_logger().info('No plan found, stopping mission')
                self.missions.remove(mission)
                return mission
            self.current_plan = plan_response.observation.split('/')
            mission.plan = self.current_plan
            self.split_plans(mission.context)
            self.send_plans_request(mission.context)

        return mission

    def fix_missions(self, mission: Mission):
        error = mission.error
        self.get_logger().info(str(error))
        error_msg = error.split("|")
        error_desc = error_msg[1].split(",")
        self.treat_error(error_desc)
        if self.should_replan:
            self.fix_plan(mission)
        else:
            self.end_simulation()

    def mark_agent_finished(self, mission: Mission, agent: str):
        for mission_robot in mission.team:
            if mission_robot.robot == agent:
                mission_robot.finished = True
                self.get_logger().info(f"Marked {agent} as finished in mission.")
                break

    def free_agent(self, agent: str):
        raise NotImplementedError("This method should be implemented by the subclass")

    def verify_mission_complete(self, mission: Mission):
        if all(mission_robot.finished for mission_robot in mission.team):
            mission.status = MissionStatus.FINISHED
            return True

        return False

    def finish_mission(self, finished_mission: Mission):
        self.get_logger().info("Mission Completed")
        self.missions.remove(finished_mission)
        self.get_logger().info(str(len(self.missions)))
        if(len(self.missions) == 0):
            self.end_simulation()

    def restart_mission(self, mission):
        raise NotImplementedError("This method should be implemented by the subclass")

    def get_agent_class(self, agent):
        # TODO only works for agents from 1 to 9
        return agent[:-1]

    def send_monitor_state_request(self, action):
        self.update_state_request = Action.Request()
        self.update_state_request.action = action
        return self.environment_client.call_async(self.update_state_request)

    def check_env(self):
        self.get_logger().info('Checking environment state')
        future = self.send_monitor_state_request(','.join(('monitor',)))
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.state = json.loads(response.observation)
        self.update_planner_state(response.observation)
        # self.verify_initial_trigger()

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

    def fix_plan(self, mission: Mission):
        context = mission.context
        team = self.get_team_from_context(context)

        self.get_logger().info('Creating plan for: %s ' % (
            ','.join(team)
        ))
        future = self.send_need_plan_request(mission.type, ','.join(team))
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

        self.get_logger().info('Plan formatted for: %s ' % (','.join(start)))

        if self.should_use_bdi:
            bdies = generate_bdi(team, formated_plan, self.mission_context, self.variables)
            for agent, rules in bdies.items():
                plans = []
                for rule in rules:
                    plans.append(rule)
                msg = String()
                msg.data = FIPAMessage(FIPAPerformative.INFORM.value, 'Coordinator', agent, 'Plan|' + '/'.join(plans)).encode()
                self.agent_publisher.publish(msg)

            for agent in team:
                msg = String()
                msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', agent, 'Start|' + ','.join(start)).encode()
                self.agent_publisher.publish(msg)

            for robot in mission.team:
                robot.plan_version = robot.plan_version + 1
                robot.current_bdi = bdies
        else:
            mission.plan = self.current_plan
            self.split_plans(team)
            self.send_plans_request(team)

        for robot in mission.team:
            robot.finished = False

    def split_plans(self, team):
        for agent in team:
            tuples = map(action_string_to_tuple, self.current_plan)
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

    def treat_error(self, error_desc):
        raise NotImplementedError("This method should be implemented by the subclass")

    def end_simulation(self):
        self.get_logger().info('Ending simulation')
        msg = Bool()
        msg.data = True
        self.end_publisher.publish(msg)
        raise SystemExit

    def send_need_plan_request(self, mission_type, team):
        self.action_request = Action.Request()
        self.action_request.action = 'need_plan|' + mission_type+ ',' + team
        return self.planner_communication_sync_client.call_async(self.action_request)

    def stop_low_priority_mission(self):
        low_priority_missions = [mission for mission in self.missions if mission.priority == 0]

        if len(low_priority_missions) == 0:
            return False

        low_mission = low_priority_missions[0]
        low_mission.status = MissionStatus.CANCELED

        for agent in low_mission.team:
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', 'Jason', 'Stop|' + agent.robot).encode()
            self.agent_publisher.publish(msg)
            self.free_agent(agent.robot)
        self.missions.remove(low_mission)

        return True

    def sync_robots(self, robots: list[MissionRobot]):
        if not robots:
            return None, []

        # Find the highest plan_version
        max_version = max(r.plan_version for r in robots)

        # Get the reference BDI from one robot with the max_version
        reference_bdi = next(r.current_bdi for r in robots if r.plan_version == max_version)

        # Collect all robots that are out of date
        outdated = [r for r in robots if r.plan_version < max_version]

        return reference_bdi, outdated

    def strip_numbers(self, name: str) -> str:
        return re.sub(r'\d+$', '', name)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            self.analyze_missions()
            # self.check_env()
