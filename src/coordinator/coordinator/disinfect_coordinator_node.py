import rclpy
from coordinator.agnostic_coordinator import AgnosticCoordinator, MissionRobot, Mission, MissionStatus
from coordinator.helper import FIPAMessage,  action_string_to_tuple, action_tuple_to_string
from std_msgs.msg import String
from coordinator.FIPAPerformatives import FIPAPerformative
from interfaces.srv import SendPlan, Action
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
        self.mission_context = "start(NurseDesinfect, NurseRoom, Spotrobot, Uvdrobot)"
        self.variables = ["Spotrobot", "NurseRoom", "NurseDesinfect", "Uvdrobot"]

class DisinfectICUMission(Mission):
    def __init__(self, team: List[MissionRobot], context):
        super().__init__(team, context)
        self.priority = 1
        self.roles = [RobotRoles.SPOT, RobotRoles.UVD, RobotRoles.NURSE]
        self.mission_context = "start(Spotrobot, NurseRoom, NurseDesinfect, Uvdrobot)"
        self.variables = ["Spotrobot", "NurseRoom", "NurseDesinfect", "Uvdrobot"]

class Coordinator(AgnosticCoordinator):
    def __init__(self):
        super().__init__('Disinfect Coordinator')
        #list of agents
        self.nurses = []
        self.spotrobot = []
        self.uvdrobot = []

        #list of agents that are currently occupied
        self.occ_nurses = []
        self.occ_spotrobots = []
        self.occ_uvdrobots = []

        #list of agents that are ready
        self.ready_spotrobots = []
        self.ready_uvdrobots = []
        self.ready_nurses = []

        # Needed for the BDI Parser
        self.mission_context = "start(Spotrobot, NurseRoom, NurseDesinfect, Uvdrobot)"
        self.variables =["Spotrobot", "NurseRoom", "NurseDesinfect", "Uvdrobot"]

        self.crr_room = None
    
    def set_agent_ready(self, decoded_msg):
        if "uvdrobot" in decoded_msg.sender:
            self.ready_uvdrobots.append(decoded_msg.sender)
            # self.get_logger().info(f"ready uvd: {len(self.ready_uvdrobots)}")
        elif "spotrobot" in decoded_msg.sender:
            self.ready_spotrobots.append(decoded_msg.sender)
            # self.get_logger().info(f"ready spot: {len(self.ready_spotrobots)}")
        elif "nurse" in decoded_msg.sender:
            self.ready_nurses.append(decoded_msg.sender)
            # self.get_logger().info(f"ready nurse: {len(self.ready_nurses)}")

    def register_agent(self, decoded_msg):
        response = None
        agent_type = decoded_msg.sender
        if 'uvdrobot' in decoded_msg.sender:
            id = str(len(self.uvdrobot) + 1)
            self.uvdrobot.append(decoded_msg.sender + id)
            # self.get_logger().info("Current uvdrobots: " + ",".join(self.uvdrobot) + " - " + id)
            
        elif 'spotrobot' in decoded_msg.sender:
            id = str(len(self.spotrobot) + 1)
            self.spotrobot.append(decoded_msg.sender + id)
            # self.get_logger().info("Current Spotrobots: " + ",".join(self.spotrobot) + " - " + id)
        elif 'nurse' in decoded_msg.sender:
            id = str(len(self.nurses) + 1)
            self.nurses.append(decoded_msg.sender + id)
            # self.get_logger().info("Current nurses: " + ",".join(self.nurses) + " - " + id)

        response = decoded_msg.sender + id
        # self.get_logger().info("Response: " + response)
        if self.should_use_bdi:
            self.register_queue.append((response, agent_type))
        # self.get_logger().info(f"colocando: {agent_type} {len(self.register_queue)}")

        return response
    
    def get_team(self) -> List[MissionRobot]:
        free_uvdrobot = list(set(self.uvdrobot) - set(self.occ_uvdrobots))
        free_spotrobot = list(set(self.spotrobot) - set(self.occ_spotrobots))
        free_nurse = list(set(self.nurses) - set(self.occ_nurses))
        
        if len(free_uvdrobot) > 0 and len(free_spotrobot) > 0 and len(free_nurse) > 0:
            team = [MissionRobot(free_uvdrobot[0]), MissionRobot(free_spotrobot[0]), MissionRobot(free_nurse[0])]
            self.occ_uvdrobots.append(free_uvdrobot[0])
            self.occ_spotrobots.append(free_spotrobot[0])
            self.occ_nurses.append(free_nurse[0])
            return team

        return None
    
    def get_start_context(self, team: List[MissionRobot]):
        mission =  (
            team[2].robot, # Nurse
            self.crr_room, # Infected Location
            team[1].robot, # spotrobot
            team[0].robot # uvdrobot
        )
        self.get_logger().info(f"context: {team[2].robot} {self.crr_room} {team[1].robot} {team[0].robot}")
        self.crr_room = None
        return mission
    
    def create_mission(self, team: List[MissionRobot], start_context):
        if 'icu' in start_context:
            mission = DisinfectICUMission(team, start_context)
        else:
            mission = DisinfectRoomMission(team, start_context)
        self.missions.append(mission)

        return mission
    
    def initial_trigger(self, msg):
        self.crr_room = msg.split('|')[1]
        self.get_logger().info(f"Initial trigger received for room: {self.crr_room}")
        self.get_logger().info("Creating mission")
        team = self.get_team()
        if(team == None):
            # self.get_logger().info("No team to start mission")
            return None

        start_context = self.get_start_context(team)

        if(start_context == None):
            # self.get_logger().info("Not possible to start mission")
            return None
        
        self.create_mission(team, start_context)

    def verify_initial_trigger(self):
        for location, disinfected in self.state['disinfected'].items():
            # Verifica se a sala da enfermeira ainda não foi desinfetada
            if not disinfected:
                # Verifica se essa sala já não está em missão
                if all(location not in mission.context for mission in self.missions):
                    self.crr_room = location
                    mission = self.start_mission()

                    if mission == None and location == 'icu': 
                        self.get_logger().info(f"ICU EMERGENCY HIGHER!!!!!!!")
                        self.get_logger().info(f"STOP ANOTHER MISSION!")
                        stop_result = self.stop_low_priority_mission()
                        if stop_result:
                            self.get_logger().info(f"Restarting!")
                            self.crr_room = location
                            self.start_mission()
                        else: 
                            self.get_logger().info(f"No mission to stop!")

                # else:
                    #self.get_logger().info(f"Room {location} already disinfected by {agent}, skipping mission.")
                    # if agent in self.occ_nurses:
                    #     for agent_context in self.current_team:
                    #         self.free_agent(agent_context)
                    #         self.get_logger().info(f"Freeing agent: {agent_context}")
                        # if  (len(self.room_queue)) > 0:
                        #     self.send_reset_request(self.current_team)
                        
                            # Verify if the mission is complete
                    # if (len(self.room_queue)) == 0:
                    #     self.verify_mission_complete(agent) 

    def get_team_from_context(self, context):
        return [context[2], context[3], context[0]]
    
    def free_agent(self, agent: str):
        if agent in self.occ_nurses:
            self.occ_nurses.remove(agent)
        elif agent in self.occ_uvdrobots:
            self.occ_uvdrobots.remove(agent)
        elif agent in self.occ_spotrobots:
            self.occ_spotrobots.remove(agent)

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



                
