import rclpy
from coordinator.agnostic_coordinator import AgnosticCoordinator

class Coordinator(AgnosticCoordinator):
    def __init__(self):
        super().__init__('Coordinator')
        # List of agents
        # NOT AGNOSTIC
        self.robots = []
        self.arms = []
        self.nurses = []
        # List of agents that are currently occupied
        # NOT AGNOSTIC
        self.occ_robots = []
        self.occ_arms = []
        self.occ_nurses = []
        # List of agents that are ready to be used
        # NOT AGNOSTIC
        self.ready_robots = []
        self.ready_arms = []
        self.ready_nurses = []
        self.nurse_queue = []
        self.mission_context = "start(Nurse, LockedDoor, Robot, ArmRoom, Arm)"
        self.variables = ["Nurse", "LockedDoor", "Robot", "ArmRoom", "Arm"]

    def set_agent_ready(self, decoded_msg):
        if "robot" in decoded_msg.sender:
            self.ready_robots.append(decoded_msg.sender)
        elif "arm" in decoded_msg.sender:
            self.ready_arms.append(decoded_msg.sender)
        elif "nurse" in decoded_msg.sender:
            self.ready_nurses.append(decoded_msg.sender)

    def register_agent(self, decoded_msg):
        response = None
        if 'arm' in decoded_msg.sender:
            id = str(len(self.arms) + 1)
            response = id
            self.arms.append(decoded_msg.sender + id)
        elif 'robot' in decoded_msg.sender:
            id = str(len(self.robots) + 1)
            response = id
            self.robots.append(decoded_msg.sender + id)
        elif 'nurse' in decoded_msg.sender:
            id = str(len(self.nurses) + 1)
            response = id
            self.nurses.append(decoded_msg.sender + id)
        return response

    def get_team(self):
        # NOT AGNOSTIC
        free_arms = list(set(self.ready_arms) - set(self.occ_arms))
        free_robots = list(set(self.ready_robots) - set(self.occ_robots))
        nurse = self.nurse_queue.pop()

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
            self.state['loc'][team[1]], # Locked Location
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
                    self.get_logger().info("Mission Completed")
                    self.end_simulation()

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
