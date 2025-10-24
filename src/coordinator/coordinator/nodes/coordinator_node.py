import rclpy
import json

from typing import List
from enum import Enum

from coordinator.helpers.agnostic_coordinator import AgnosticCoordinator, MissionRobot, Mission, MissionStatus, RobotStatus

class RobotRoles(Enum):
    SPOT = 1
    UVD = 2
    NURSE = 3
    COLLECTOR = 4
    ARM = 5

class DisinfectRoomMission(Mission):
    def __init__(self, team: List[MissionRobot], context):
        super().__init__(team, context)
        self.priority = 0
        self.roles = [RobotRoles.SPOT, RobotRoles.UVD, RobotRoles.NURSE]
        self.mission_context = "start(SpotRobot, NurseRoom, Nurse, UvdRobot)"
        self.variables = ["SpotRobot", "NurseRoom", "Nurse", "UvdRobot"]
        self.room = None
        self.type = 'DisinfectRoomMission'

class DisinfectICUMission(Mission):
    def __init__(self, team: List[MissionRobot], context):
        super().__init__(team, context)
        self.priority = 2
        self.roles = [RobotRoles.SPOT, RobotRoles.UVD, RobotRoles.NURSE]
        self.mission_context = "start(SpotRobot, NurseRoom, Nurse, UvdRobot)"
        self.variables = ["SpotRobot", "NurseRoom", "Nurse", "UvdRobot"]
        self.room = None
        self.type = 'DisinfectICUMission'

class CollectSampleMission(Mission):
    def __init__(self, team: List[MissionRobot], context):
        super().__init__(team, context)
        self.priority = 1
        self.roles = [RobotRoles.COLLECTOR, RobotRoles.ARM, RobotRoles.NURSE]
        self.mission_context = "start(Nurse, NurseRoom, Collector, ArmRoom, Arm)"
        self.variables = ["Nurse", "NurseRoom", "Collector", "ArmRoom", "Arm"]
        self.room = None
        self.type = 'CollectSampleMission'

class Coordinator(AgnosticCoordinator):
    def __init__(self):
        """
        1.1. Initialize specific coordinator variables (state)
        """
        super().__init__('Disinfect Coordinator')

        self.state = {
            'loc': {
                'nurse1': 'nr',
                'nurse2': 'nr',
                'nurse3': 'nr',
                'nurse4': 'nr',
                'uvd1': 'ds',
                'spot1': 'ds',
                'uvd2': 'ds',
                'spot2': 'ds',
                'collector1': 'ds',
                'collector2': 'ds',
                'arm1': 'lab'
            },
            'doors': {
                'room1': True,
                'room2': True,
                'room3': True,
                'room4': True,
                'room5': True,
                'room6': True,
                'lab': True,
                'nr': True,
                'icu': True
            },
            'sample': {
                'room1': False,
                'room2': False,
                'room3': False,
                'room4': False,
                'room5': False,
                'room6': False,
                'nurse1': False,
                'nurse2': False,
                'nurse3': False,
                'nurse4': False,
                'uvdrobot1': False,
                'spotrobot1': False,
                'uvdrobot2': False,
                'spotrobot2': False,
                'collector1': False,
                'collector2': False,
                'arm1': False
            },
            'cleaned': {
                'room1': True,
                'room2': True,
                'room3': True,
                'room4': True,
                'icu': True
            },
            'samples': {
                'room1': False,
                'room2': False,
                'room3': False,
                'room4': False,
                'room5': False,
                'room6': False,
                'icu': False
            },
            'disinfected': {
                'room1': True,
                'room2': True,
                'room3': True,
                'room4': True,
                'room5': True,
                'room6': True,
                'icu': True
            },
            'low_battery': {
                'uvd1': False,
                'spot1': False,
                'uvd2': False,
                'spot2': False,
                'collector1': False,
                'collector2': False,
                'arm1': False
            }
        }

    def register_agent(self, decoded_msg):
        """3. Register agents by creating the MissionRobot with its respective specific RobotRole"""
        agent_name = decoded_msg.sender
        robot = MissionRobot(agent_name)

        if 'uvd' in agent_name:
            robot.role = RobotRoles.UVD
        elif 'spot' in agent_name:
            robot.role = RobotRoles.SPOT
        elif 'nurse' in agent_name:
            robot.role = RobotRoles.NURSE  
        elif 'collector' in agent_name:
            robot.role = RobotRoles.COLLECTOR  
        elif 'arm' in agent_name:
            robot.role = RobotRoles.ARM           
            
        self.robots.append(robot)

        return 'success'

    def initial_trigger(self, decoded_msg):
        """
        4. At some time, the Coordinator will receive the initial triger
        It will create a new mission for it.
        """

        message = decoded_msg.content.split('|')[1]

        mission_type = message.split(',')[0]
        room = message.split(',')[1]

        mission = self.create_mission(mission_type, room)
        team = self.get_team(mission)

        if(team is not None):
            mission.team = team
            mission.context = self.get_start_context(mission_type, team, room)
        else:
            mission.status = MissionStatus.WAITING_TEAM

        if(mission_type == 'Disinfect'):
            self.state['loc'][decoded_msg.sender] = room
            self.state['disinfected'][room] = False
        elif (mission_type == 'Sample'):
            self.state['sample'][room] = True

        self.missions.append(mission)

    def create_mission(self, mission_type, trigger):
        self.get_logger().info(f"Creating mission with team")

        if(mission_type == 'Disinfect' and trigger == 'icu'):
            mission = DisinfectICUMission([], {})
        elif(mission_type == 'Disinfect' and not trigger == 'icu'):
            mission = DisinfectRoomMission([], {})
        elif (mission_type == 'Sample'):
            mission = CollectSampleMission([], {})

        mission.trigger = trigger
        return mission

    def get_start_context(self, mission_type, team: List[MissionRobot], room: str):
        if(mission_type == 'Disinfect'):
            context = (
                team[2].robot, # Nurse
                room, # Infected Location
                team[0].robot, # spotrobot
                team[1].robot # uvdrobot
            )
        elif (mission_type == 'Sample'):
            context =  (
                team[2].robot, # Nurse
                room,
                team[0].robot, # Robot
                team[1].robot # Arm
            )
        self.get_logger().info(f"context: {str(context)}")
        return context

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
