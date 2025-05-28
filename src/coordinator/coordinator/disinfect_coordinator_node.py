import rclpy
from coordinator.agnostic_coordinator import AgnosticCoordinator
from coordinator.helper import FIPAMessage,  action_string_to_tuple, action_tuple_to_string
from std_msgs.msg import String
from coordinator.FIPAPerformatives import FIPAPerformative
from interfaces.srv import SendPlan, Action

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
        '''
        self.ready_spotrobots = []
        self.ready_uvdrobots = []
        self.ready_nurses = []
        '''
        #list of dirty rooms
        self.room_queue = []
        self.mission_context = "start(NurseDesinfect, NurseRoom, uvdrobot, spotrobot)"
        self.variables = ["NurseDesinfect", "NurseRoom", "uvdrobot", "spotrobot"]
        self.current_plan = []
        self.plans_actions = 0
        self.plans_actions_current = 0
        self.agents_actions = {}
    '''
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
    '''



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
        '''
        if "uvdrobot" in decoded_msg.sender:
            self.ready_uvdrobots.append(decoded_msg.sender + id)
        elif "spotrobot" in decoded_msg.sender:
            self.ready_spotrobots.append(decoded_msg.sender + id) 
        elif "nurse" in decoded_msg.sender:
            self.ready_nurses.append(decoded_msg.sender + id)
        '''
        return response
    
    def get_team(self):
        free_uvdrobot = list(set(self.uvdrobot) - set(self.occ_uvdrobots))
        free_spotrobot = list(set(self.spotrobot) - set(self.occ_spotrobots))
        #self.get_logger().info(f"Tamanho da room_queue: {len(self.room_queue)}")
        # TODO: Implement nurse queue
        if len(self.room_queue) == 0:
            return None
        nurse = None
        
        for potential_nurse in self.room_queue:
            if potential_nurse in self.nurses:
                nurse = potential_nurse
                break

        if nurse is None:
            return None
        

        if len(free_uvdrobot) > 0 and len(free_spotrobot) > 0:
            team = (free_uvdrobot[0], free_spotrobot[0], nurse)
            self.occ_uvdrobots.append(free_uvdrobot[0])
            self.occ_spotrobots.append(free_spotrobot[0])
            self.occ_nurses.append(nurse)
            return team

        return None
    
    def get_start_context(self, team):
        mission =  (
            team[2], # Nurse
            self.state['loc'][team[2]], # Locked Location
            team[1], # spotrobot
            team[0] # uvdrobot
        )
        self.get_logger().info(f"context: {team[2]} {self.state['loc'][team[2]]} {team[1]} {team[0]}")
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
        return [context[2], context[3],context[0]]
    
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
    
    def start_mission(self):
        self.get_logger().info("Starting mission")
        if len(self.room_queue) == 0:
            self.get_logger().info("No more rooms to disinfect")
            return
        team = self.get_team()
        if team is None:
            self.get_logger().info("No available team to start mission")
            return

        context = self.get_start_context(team)
        self.get_logger().info(f"Starting mission with context: {context}")

        new_team = self.get_team_from_context(context)
        self.get_logger().info('Creating plan for: %s ' % (
            ','.join(new_team)
        ))
        future = self.send_need_plan_request(','.join(new_team))
        rclpy.spin_until_future_complete(self, future)
        plan_response = future.result()
        self.get_logger().info('Plan received for: %s ' % (
            ','.join(new_team)
        ))

        self.get_logger().info(plan_response.observation)
        self.current_plan = plan_response.observation.split('/')

        tuples = list(map(action_string_to_tuple, self.current_plan))

        for agent in team:
            self.get_logger().info(f'Splitting plans for agent: {agent}')
            self.agents_actions[agent] = []
            for action in tuples:
                if agent in action:
                    self.get_logger().info(f'Adding action {action} to agent {agent}')
                    self.agents_actions[agent].append(action)
        
        self.send_plans_request(team)
        #for action in self.current_plan:
            # split plan between agents

        # send plan to agents

        # Remove the room from the queue
        self.room_queue.remove(team[2])
    
    

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



                
