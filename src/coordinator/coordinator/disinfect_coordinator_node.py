import rclpy
from coordinator.agnostic_coordinator import AgnosticCoordinator
from coordinator.helper import FIPAMessage
from std_msgs.msg import String
from coordinator.FIPAPerformatives import FIPAPerformative


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
        #list of dirty rooms
        self.room_queue = []
        self.mission_context = "start(NurseDesinfect, NurseRoom, uvdrobot, spotrobot)"
        self.variables = ["NurseDesinfect", "NurseRoom", "uvdrobot", "spotrobot"]

    def set_agent_ready(self, decoded_msg):
        if "uvdrobot" in decoded_msg.sender:
            self.ready_uvdrobots.append(decoded_msg.sender)
            self.get_logger().info(f"ready uvd: {len(self.ready_uvdrobots)}")
        elif "spotrobot" in decoded_msg.sender:
            self.ready_spotrobots.append(decoded_msg.sender)
            self.get_logger().info(f"ready spot: {len(self.ready_spotrobots)}")
        elif "nurse" in decoded_msg.sender:
            self.ready_nurses.append(decoded_msg.sender)
            self.get_logger().info(f"ready 123: {len(self.ready_nurses)}")

    def register_agent(self, decoded_msg):
        response = None
        agent_type = decoded_msg.sender
        if 'uvdrobot' in decoded_msg.sender:
            id = str(len(self.uvdrobot) + 1)
            self.uvdrobot.append(decoded_msg.sender + id)
            self.get_logger().info("Current uvdrobots: " + ",".join(self.uvdrobot) + " - " + id)
            
        elif 'spotrobot' in decoded_msg.sender:
            id = str(len(self.spotrobot) + 1)
            self.spotrobot.append(decoded_msg.sender + id)
            self.get_logger().info("Current Spotrobots: " + ",".join(self.spotrobot) + " - " + id)
        elif 'nurse' in decoded_msg.sender:
            id = str(len(self.nurses) + 1)
            self.nurses.append(decoded_msg.sender + id)
            self.get_logger().info("Current nurses: " + ",".join(self.nurses) + " - " + id)

        response = decoded_msg.sender + id
        self.get_logger().info("Response: " + response)
        self.register_queue.append((response, agent_type))

        if "uvdrobot" in decoded_msg.sender:
            self.ready_uvdrobots.append(decoded_msg.sender + id)
        elif "spotrobot" in decoded_msg.sender:
            self.ready_spotrobots.append(decoded_msg.sender + id) 
        elif "nurse" in decoded_msg.sender:
            self.ready_nurses.append(decoded_msg.sender + id)
            
        return response
    
    def get_team(self):
        free_uvdrobot = list(set(self.ready_uvdrobots) - set(self.occ_uvdrobots))
        free_spotrobot = list(set(self.ready_spotrobots) - set(self.occ_spotrobots))
        #self.get_logger().info(f"Tamanho da room_queue: {len(self.room_queue)}")
        # TODO: Implement nurse queue
        if len(self.room_queue) == 0:
            return None
        nurse = None
        
        for potential_nurse in self.room_queue:
            if potential_nurse in self.ready_nurses:
                nurse = potential_nurse
                break

        if nurse is None:
            return None
        

        if len(free_uvdrobot) > 0 and len(free_spotrobot) > 0:
            team = (free_uvdrobot[0], free_spotrobot[0], nurse)
            self.occ_uvdrobots.append(free_uvdrobot[0])
            self.occ_spotrobots.append(free_spotrobot[0])
            self.occ_nurses.append(nurse)
            self.get_logger().info(f"teste3 {free_uvdrobot[0]} {free_spotrobot[0]} {nurse}")
            return team

        return None
    
    def get_start_context(self, team):
        mission =  (
            team[2], # Nurse
            self.state['loc'][team[2]], # Locked Location
            team[1], # spotrobot
            team[0] # uvdrobot
        )
        self.get_logger().info(f"context: {team[2]} {self.state['loc'][team[2]]} { team[1]} {team[0]}")
        self.missions.append(mission)
        return mission
    
    def verify_initial_trigger(self):
        for agent, location in self.state['loc'].items():
            if 'nurse' in agent:
                # Verifica se a sala da enfermeira ainda não foi desinfetada
                if not self.state['disinfected'][location]:
                    # Verifica se essa sala já não está em missão
                    if all(agent not in team for team in self.missions):
                        self.room_queue.append(agent)
                        self.start_mission()

    def get_team_from_context(self, context):
        return [context[0], context[2], context[3]]
    
    def free_agent(self, agent):
        if agent in self.occ_nurses:
            self.occ_nurses.remove(agent)
        elif agent in self.occ_uvdrobots:
            self.occ_uvdrobots.remove(agent)
        elif agent in self.occ_spotrobots:
            self.occ_spotrobots.remove(agent)

    def verify_mission_complete(self, agent):
        # Find any mission containing this agent
        for mission in self.missions:
            if agent in mission:
                # Check if all agents in this mission are now free
                all_free = True
                for mission_agent in mission:
                    if (mission_agent in self.occ_nurses or 
                        mission_agent in self.occ_uvdrobots or
                        mission_agent in self.occ_spotrobots):
                        all_free = False
                        break
                # If all agents are free, remove the mission
                if all_free:
                    self.missions.remove(mission)
                    for agent in mission:
                        msg = String()
                        msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, agent, 'Jason', 'End|' + agent).encode()
                        self.jason_publisher.publish(msg)
                    self.get_logger().info("Mission Completed")
                    self.end_simulation()

    def idk(self, mission, error):
        return
    

def main():
    rclpy.init()
    coordinator = Coordinator()
    coordinator.get_logger().info('spin')
    try:
        coordinator.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    coordinator.get_logger().info('stop spin')
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



                
