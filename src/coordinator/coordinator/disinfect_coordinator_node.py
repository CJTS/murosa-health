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
        
        self.ready_spotrobots = []
        self.ready_uvdrobots = []
        self.ready_nurses = []
        
        #list of dirty rooms
        self.room_queue = []
        self.mission_context = "start(NurseDesinfect, NurseRoom, Spotrobot, Uvdrobot)"
        #self.variables = ["NurseDesinfect", "NurseRoom", "uvdrobot", "spotrobot"]
        self.variables =["Spotrobot", "NurseRoom", "NurseDesinfect", "Uvdrobot"]
        self.current_plan = []
        self.current_team = []	
        self.agents_actions = {}


        #self.reset_publisher = self.create_publisher(String, '/coordinator/agent/reset', 10)
    
    def set_agent_ready(self, decoded_msg):
        if "uvdrobot" in decoded_msg.sender:
            self.ready_uvdrobots.append(decoded_msg.sender)
            self.get_logger().info(f"ready uvd: {len(self.ready_uvdrobots)}")
        elif "spotrobot" in decoded_msg.sender:
            self.ready_spotrobots.append(decoded_msg.sender)
            self.get_logger().info(f"ready spot: {len(self.ready_spotrobots)}")
        elif "nurse" in decoded_msg.sender:
            self.ready_nurses.append(decoded_msg.sender)
            self.get_logger().info(f"ready nurse: {len(self.ready_nurses)}")
    



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
        self.get_logger().info(f"colocando: {agent_type} {len(self.register_queue)}")
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
                        if agent not in self.room_queue:
                            self.room_queue.append(agent)
                        self.start_mission()
                else:
                    #self.get_logger().info(f"Room {location} already disinfected by {agent}, skipping mission.")
                    if agent in self.occ_nurses:
                        for agent_context in self.current_team:
                            self.free_agent(agent_context)
                            self.get_logger().info(f"Freeing agent: {agent_context}")
                        if  (len(self.room_queue)) > 0:
                            self.send_reset_request(self.current_team)
                        
                            # Verify if the mission is complete
                    if (len(self.room_queue)) == 0:
                        self.verify_mission_complete(agent)
                    

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
        #self.send_reset_request(self.current_team)
        return
    '''
    def start_mission(self):
        self.agents_actions = {}
        #self.get_logger().info("Starting mission")
        if len(self.room_queue) == 0:
            self.get_logger().info("No more rooms to disinfect")
            return
        team = self.get_team()
        if team is None:
            #self.get_logger().info("No available team to start mission")
            return

        context = self.get_start_context(team)
        self.get_logger().info(f"Starting mission with context: {context}")

        new_team = self.get_team_from_context(context)
        self.current_team = new_team
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
            self.agents_actions[agent] = []
            for action in tuples:
                if agent in action:
                    self.agents_actions[agent].append(action)
                    
        
        self.send_plans_request(team)
        #for action in self.current_plan:
            # split plan between agents

        # send plan to agents

        # Remove the room from the queue
        self.room_queue.remove(team[2])


    def send_update_uncleaned_room_request(self, room):
        self.get_logger().info(f"Updating uncleaned room: {room}")
        self.action_request = Action.Request()
        self.action_request.action = ('|'.join(('update_room_uncleaned', room))) 
        future = self.planner_communication_sync_client.call_async(self.action_request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(response.observation)

    def fix_plan(self, context):
        self.send_update_uncleaned_room_request(context[1])
        team = self.get_team_from_context(context)

        self.get_logger().info('Creating new plan for: %s ' % (
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
        self.current_team = team
        tuples = list(map(action_string_to_tuple, self.current_plan))

        for agent in team:
            self.agents_actions[agent] = []
            for action in tuples:
                if agent in action:
                    self.agents_actions[agent].append(action)
        self.send_plans_request(team)

    def send_reset_request(self, team):
        self.get_logger().info('Sending reset requests...')
        for agent in team:
            reset_msg = String()
            reset_msg.data = FIPAMessage(
                FIPAPerformative.REQUEST.value,
                'Coordinator',
                agent,
                'Reset'
            ).encode()
            self.reset_publisher.publish(reset_msg)

        self.get_logger().info('Reset requests sent.')

    '''
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



                
