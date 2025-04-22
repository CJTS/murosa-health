import random
import rclpy
from coordinator.agnostic_coordinator import AgnosticCoordinator

class Coordinator(AgnosticCoordinator):
    def __init__(self):
        super().__init__('Patrol Coordinator')
        self.patrols = []
        self.visiting_wps = []
        self.visited_wps = []
        self.occ_patrols = []
        self.ready_patrols = []
        self.mission_context = "start(Patrol, Base, Room1)"
        self.variables = ["Patrol", "Room1", "Base"]

    def set_agent_ready(self, decoded_msg):
        if "patrol" in decoded_msg.sender:
            self.ready_patrols.append(decoded_msg.sender)


    def register_agent(self, decoded_msg):
        response = None
        agent_type = decoded_msg.sender
        name = self.make_agent_id(agent_type)
        response = name

        if 'patrol' in decoded_msg.sender:
            self.patrols.append(name)

        self.register_queue.append((name, agent_type))
        return response

    def get_team(self):
        free_patrols = list(set(self.ready_patrols) - set(self.occ_patrols))

        if len(free_patrols) > 0:
            team = [free_patrols[0]]
            self.occ_patrols.append(free_patrols[0])
            return team

        return None

    def get_start_context(self, team):
        rooms = [wp for wp, patrolled in self.state['patrolled'].items() 
                if not patrolled and wp not in self.visited_wps and wp not in self.visiting_wps]
        if len(rooms) > 0:
            room = random.choice(rooms)
            self.visiting_wps.append(room)
            return (team[0], "wp_control", room)

        return None

    def verify_initial_trigger(self):
        for wp, patrolled in self.state['patrolled'].items():
            if patrolled and wp in self.visiting_wps:
                self.visiting_wps.remove(wp)
                self.visited_wps.append(wp)

        # Check if at least one waypoint has not been patrolled
        if any(not patrolled for patrolled in self.state['patrolled'].values()):
            self.start_mission()
        else:
            self.get_logger().info('All waypoints have been patrolled')
            self.get_logger().info("Mission Completed")
            self.end_simulation()

    def get_team_from_context(self, context):
        return context

    def free_agent(self, agent):
        self.occ_patrols.remove(agent)

    def verify_mission_complete(self, agent):
        # Find any mission containing this agent
        for mission in self.missions:
            if agent in mission:
                # Check if all agents in this mission are now free
                all_free = True
                for mission_agent in mission:
                    if mission_agent in self.occ_patrols:
                        all_free = False
                        break
                
                # If all agents are free, remove the mission
                if all_free:
                    self.missions.remove(mission)
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
