import rclpy
from coordinator.agnostic_coordinator import AgnosticCoordinator
from coordinator.helper import FIPAMessage
from std_msgs.msg import String
from coordinator.FIPAPerformatives import FIPAPerformative

class Coordinator(AgnosticCoordinator):
    def __init__(self):
        super().__init__('Coordinator')
        # List of agents
        self.robots = []
        self.arms = []
        self.nurses = []
        # List of agents that are currently occupied
        self.occ_robots = []
        self.occ_arms = []
        self.occ_nurses = []
        # List of agents that are ready to be used
        self.ready_robots = []
        self.ready_arms = []
        self.ready_nurses = []
        self.nurse_queue = []
        self.mission_context = "start(Nurse, NurseRoom, Robot, ArmRoom, Arm)"
        self.variables = ["Nurse", "NurseRoom", "Robot", "ArmRoom", "Arm"]

    def set_agent_ready(self, decoded_msg):
        if "robot" in decoded_msg.sender:
            self.ready_robots.append(decoded_msg.sender)
        elif "arm" in decoded_msg.sender:
            self.ready_arms.append(decoded_msg.sender)
        elif "nurse" in decoded_msg.sender:
            self.ready_nurses.append(decoded_msg.sender)

    def register_agent(self, decoded_msg):
        response = None
        agent_type = decoded_msg.sender

        if 'arm' in decoded_msg.sender:
            id = str(len(self.arms) + 1)
            self.arms.append(decoded_msg.sender + id)
            self.get_logger().info("Current arms: " + ",".join(self.arms) + " - " + id)
        elif 'robot' in decoded_msg.sender:
            id = str(len(self.robots) + 1)
            self.robots.append(decoded_msg.sender + id)
            self.get_logger().info("Current robots: " + ",".join(self.robots) + " - " + id)
        elif 'nurse' in decoded_msg.sender:
            id = str(len(self.nurses) + 1)
            self.nurses.append(decoded_msg.sender + id)
            self.get_logger().info("Current nurses: " + ",".join(self.nurses) + " - " + id)

        response = decoded_msg.sender + id
        self.get_logger().info("Response: " + response)
        self.register_queue.append((response, agent_type))
        return response

    def get_team(self):
        free_arms = list(set(self.ready_arms) - set(self.occ_arms))
        free_robots = list(set(self.ready_robots) - set(self.occ_robots))
        # TODO: Implement nurse queue
        if len(self.nurse_queue) == 0:
            return None

        nurse = None
        
        for potential_nurse in self.nurse_queue:
            if potential_nurse in self.ready_nurses:
                nurse = potential_nurse
                break

        if nurse is None:
            return None

        if len(free_arms) > 0 and len(free_robots) > 0:
            team = (free_arms[0], free_robots[0], nurse)
            self.occ_arms.append(free_arms[0])
            self.occ_robots.append(free_robots[0])
            self.occ_nurses.append(nurse)
            return team

        return None

    def get_start_context(self, team):
        mission =  (
            team[2], # Nurse
            self.state['loc'][team[2]], # Locked Location
            team[1], # Robot
            self.state['loc'][team[0]], # Arm Location
            team[0] # Arm
        )
        self.missions.append(mission)
        return mission

    def verify_initial_trigger(self):
        for key, value in self.state['sample'].items():
            if value and all(key not in team for team in self.missions) and 'nurse' in key:
                self.nurse_queue.append(key)
                self.start_mission()

    def get_team_from_context(self, context):
        return [context[0], context[2], context[4]]

    def free_agent(self, agent):
        if agent in self.occ_nurses:
            self.occ_nurses.remove(agent)
        elif agent in self.occ_robots:
            self.occ_robots.remove(agent)
        elif agent in self.occ_arms:
            self.occ_arms.remove(agent)

    def verify_mission_complete(self, agent):
        # Find any mission containing this agent
        for mission in self.missions:
            if agent in mission:
                # Check if all agents in this mission are now free
                all_free = True
                for mission_agent in mission:
                    if (mission_agent in self.occ_nurses or 
                        mission_agent in self.occ_robots or
                        mission_agent in self.occ_arms):
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
