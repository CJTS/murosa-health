import rclpy
import json
import time
from coordinator.coordinator.helpers.agnostic_coordinator import AgnosticCoordinator, MissionRobot, Mission, MissionStatus, RobotStatus
from coordinator.helper import FIPAMessage
from std_msgs.msg import String
from coordinator.FIPAPerformatives import FIPAPerformative
from typing import List
from enum import Enum

class RobotRoles(Enum):
    SPOT = 1
    UVD = 2
    NURSE = 3

class DisinfectRoomMission(Mission):
    def __init__(self, team: List[MissionRobot], context):
        super().__init__(team, context)
        self.priority = 0
        self.roles = [RobotRoles.SPOT, RobotRoles.UVD, RobotRoles.NURSE]
        self.mission_context = "start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot)"
        self.variables = ["SpotRobot", "NurseDisinfectRoom", "NurseDisinfect", "UvdRobot"]
        self.room = None

class DisinfectICUMission(Mission):
    def __init__(self, team: List[MissionRobot], context):
        super().__init__(team, context)
        self.priority = 1
        self.roles = [RobotRoles.SPOT, RobotRoles.UVD, RobotRoles.NURSE]
        self.mission_context = "start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot)"
        self.variables = ["SpotRobot", "NurseDisinfectRoom", "NurseDisinfect", "UvdRobot"]
        self.room = None

class Coordinator(AgnosticCoordinator):
    def __init__(self):
        super().__init__('Disinfect Coordinator')

        #list of agents
        self.nurses: List[MissionRobot] = []
        self.spotrobot: List[MissionRobot] = []
        self.uvdrobot: List[MissionRobot] = []

        # Needed for the BDI Parser
        self.mission_context = "start(SpotRobot, NurseDisinfectRoom, NurseDisinfect, UvdRobot)"
        self.variables =["SpotRobot", "NurseDisinfectRoom", "NurseDisinfect", "UvdRobot"]

        self.state = {
            'loc': { 
                'nurse_disinfected1': 'room1',
                'nurse_disinfected2': 'room2', 
                'nurse_disinfected3': 'room3',
                'nurse_disinfected4': 'room4',
                'uvdrobot1': 'room4', 
                'spotrobot1': 'room4',
                'uvdrobot2': 'room4', 
                'spotrobot2': 'room4'
            },
            'doors': { 
                'room1': True, 
                'room2': True, 
                'room3': True, 
                'room4': True, 
                'icu': True
            },
            'cleaned': { 
                'room1': True,
                'room2': True,
                'room3': True,
                'room4': True,
                'icu': True
            },
            'disinfected': {
                'room1': True,
                'room2': True,
                'room3': True,
                'room4': True,
                'icu': True
            },
            'low_battery': {
                'uvdrobot1': False,
                'spotrobot1': False,
                'uvdrobot2': False,
                'spotrobot2': False
            }
        }
    
    def set_agent_ready(self, decoded_msg):
        if "uvdrobot" in decoded_msg.sender:
            robot = next((robot for robot in self.uvdrobot if robot.robot == decoded_msg.sender), None)
            robot.status = RobotStatus.READY
        elif "spotrobot" in decoded_msg.sender:
            robot = next((robot for robot in self.spotrobot if robot.robot == decoded_msg.sender), None)
            robot.status = RobotStatus.READY
        elif "nurse" in decoded_msg.sender:
            robot = next((robot for robot in self.nurses if robot.robot == decoded_msg.sender), None)
            robot.status = RobotStatus.READY

    def register_agent(self, decoded_msg):
        response = None
        agent_type = decoded_msg.sender
        if 'uvdrobot' in decoded_msg.sender:
            id = str(len(self.uvdrobot) + 1)
            robot = MissionRobot(decoded_msg.sender + id)
            robot.role = RobotRoles.UVD
            self.uvdrobot.append(robot)
        elif 'spotrobot' in decoded_msg.sender:
            id = str(len(self.spotrobot) + 1)
            robot = MissionRobot(decoded_msg.sender + id)
            robot.role = RobotRoles.SPOT
            self.spotrobot.append(robot)
        elif 'nurse' in decoded_msg.sender:
            id = str(len(self.nurses) + 1)
            robot = MissionRobot(decoded_msg.sender + id)
            robot.role = RobotRoles.NURSE
            self.nurses.append(robot)

        response = decoded_msg.sender + id
        if self.should_use_bdi:
            self.register_queue.append((response, agent_type))

        return response
    
    def get_team(self, trigger) -> List[MissionRobot]:
        free_uvdrobot = next((robot for robot in self.uvdrobot if robot.status == RobotStatus.READY), None)
        free_spotrobot = next((robot for robot in self.spotrobot if robot.status == RobotStatus.READY), None)

        time.sleep(1)
        
        if free_uvdrobot is not None and free_spotrobot is not None:
            nurse_name = next(
                (agent for agent, location in self.state['loc'].items()
                if location == trigger and agent.startswith("nurse")),
                None  # default if not found
            )
            nurse = next((robot for robot in self.nurses if robot.robot == nurse_name), None)

            team = [free_uvdrobot, free_spotrobot, nurse]

            all_same_version = all(agent.plan_version == team[0].plan_version for agent in team)

            if(all_same_version):
                free_uvdrobot.status = RobotStatus.OCCUPIED
                free_spotrobot.status = RobotStatus.OCCUPIED
                nurse.status = RobotStatus.OCCUPIED
                return team
            else:
                ref_bdi, outdated = self.sync_robots(team)

                for outdated_agent in outdated:
                    for agent, rules in ref_bdi.items():
                        if(self.strip_numbers(agent) == self.strip_numbers(outdated_agent.robot)):
                            plans = []
                            for rule in rules:
                                plans.append(rule)
                            msg = String()
                            msg.data = FIPAMessage(FIPAPerformative.INFORM.value, 'Coordinator', outdated_agent.robot, 'Plan|' + '/'.join(plans)).encode()
                            self.agent_publisher.publish(msg)

                time.sleep(1)

                free_uvdrobot.status = RobotStatus.OCCUPIED
                free_spotrobot.status = RobotStatus.OCCUPIED
                nurse.status = RobotStatus.OCCUPIED
                return team
        return None
    
    def get_start_context(self, team: List[MissionRobot], room: str):
        mission =  (
            team[1].robot, # Nurse
            room, # Infected Location
            team[2].robot, # spotrobot
            team[0].robot # uvdrobot
        )
        self.get_logger().info(f"context: {team[1].robot} {room} {team[2].robot} {team[0].robot}")
        room = None
        return mission
    
    def create_mission(self, team: List[MissionRobot], start_context, room):
        self.get_logger().info(f"Creating mission with team")
        if room == 'icu':
            mission = DisinfectICUMission(team, start_context)
        else:
            mission = DisinfectRoomMission(team, start_context)
        mission.trigger = room
        self.missions.append(mission)

        return mission
    
    def create_empty_mission(self, room):
        self.get_logger().info(f"Creating mission without team")
        if room == 'icu':
            mission = DisinfectICUMission([], {})
        else:
            mission = DisinfectRoomMission([], {})
    
        mission.status = MissionStatus.WAITING_TEAM
        mission.trigger = room
        self.missions.append(mission)
    
        return mission
    
    def initial_trigger(self, decoded_msg):
        room = decoded_msg.content.split('|')[1]
        self.state['loc'][decoded_msg.sender] = room
        self.state['disinfected'][room] = False

        self.get_logger().info(f"Initial trigger received for room: {room}")
        team = self.get_team(room)

        if(team == None):
            if room == 'icu':
                self.get_logger().info("ERROR|icu_room")
                self.stop_low_priority_mission()
                time.sleep(1)
                team = self.get_team(room)
                if(team == None):
                    self.get_logger().info("Cant make team")
                    return
                start_context = self.get_start_context(team, room)
                self.create_mission(team, start_context, room)
            self.create_empty_mission(room)
            return

        start_context = self.get_start_context(team, room)
        self.create_mission(team, start_context, room)

    def get_team_from_context(self, context):
        self.get_logger().info(f"Getting team from context: {context}")
        return [context[0], context[3], context[2]]
    
    def free_agent(self, agent: str):
        for mission in self.missions:
            for mission_robot in mission.team:
                if mission_robot.robot == agent:
                    mission_robot.status = RobotStatus.READY
                    return

    def treat_error(self, error_desc):
        if error_desc[0] == 'dirty_room':
            room = error_desc[1]
            self.state['cleaned'][room] = False
            self.get_logger().info(f"Room {room} marked as dirty again")
        elif error_desc[0] == 'closed_door':
            door = error_desc[1]
            self.state['doors'][door] = False
            self.get_logger().info(f"Door {door} marked as closed")
        elif error_desc[0] == 'low_battery':
            robot = error_desc[1]
            self.state['low_battery'][robot] = True
            self.get_logger().info(f"Robot {robot} marked as low battery")
        self.update_planner_state(json.dumps(self.state))

def main():
    rclpy.init()
    coordinator = Coordinator()
    try:
        coordinator.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    coordinator.get_logger().info('stop spin')
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



                
