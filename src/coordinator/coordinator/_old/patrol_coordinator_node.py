import random
import rclpy
from coordinator.agnostic_coordinator import AgnosticCoordinator
from std_msgs.msg import String
from coordinator.helper import FIPAMessage
from coordinator.FIPAPerformatives import FIPAPerformative

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
        self.known_errors.append("low_battery")

    def set_agent_ready(self, decoded_msg):
        if "patrol" in decoded_msg.sender:
            self.ready_patrols.append(decoded_msg.sender)

    def register_agent(self, decoded_msg):
        response = None
        agent_type = decoded_msg.sender

        if 'patrol' in decoded_msg.sender:
            id = str(len(self.patrols) + 1)
            self.patrols.append(decoded_msg.sender + id)

        response = decoded_msg.sender + id
        self.register_queue.append((response, agent_type))
        return response

    def get_team(self):
        free_patrols = list(set(self.ready_patrols) - set(self.occ_patrols))

        if len(free_patrols) > 0:
            self.get_logger().info("Adding robot: " + free_patrols[0])
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
            mission = (team[0], "wp_control", room)
            self.missions.append(mission)
            return mission

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
        return [context[0]]

    def free_agent(self, agent):
        self.get_logger().info(agent)
        self.get_logger().info("Agent: " + agent + " removed from: " +",".join(self.occ_patrols))
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

    def restart_mission(self, context):
        team = self.get_team_from_context(context)
        for agent in team:
            self.get_logger().info('Sending restart to:' + agent)
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', agent, 'Start|' + ','.join(context)).encode()
            self.agent_publisher.publish(msg)

    def idk(self, mission, error):
        error_msg = error.split("|")
        error_desc = error_msg[1].split(",")
        wp = error_desc[2]
        if wp in self.visiting_wps:
            self.visiting_wps.remove(wp)
        return

def main():
    rclpy.init()
    coordinator = Coordinator()
    coordinator.get_logger().info('spin, Patrol')
    try:
        coordinator.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    coordinator.get_logger().info('stop spin')
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
