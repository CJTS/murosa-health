import rclpy
from coordinator.agnostic_coordinator import AgnosticCoordinator

class Coordinator(AgnosticCoordinator):
    def __init__(self):
        super().__init__('Patrol Coordinator')
        self.patrols = []
        self.occ_patrols = []
        self.ready_patrols = []
        self.mission_context = "!patrol"
        self.variables = []

    def set_agent_ready(self, decoded_msg):
        if "patrol" in decoded_msg.sender:
            self.ready_patrols.append(decoded_msg.sender)

    def register_agent(self, decoded_msg):
        response = None
        if 'patrol' in decoded_msg.sender:
            id = str(len(self.patrols) + 1)
            response = id
            self.patrols.append(decoded_msg.sender + id)
        return response

    def get_team(self):
        free_patrols = list(set(self.ready_patrols) - set(self.occ_patrols))

        if len(free_patrols) > 0:
            team = [free_patrols[0]]
            self.occ_patrols.append(free_patrols[0])
            return team

        return None

    def get_start_context(self, team):
        return team

    def verify_initial_trigger(self):
        self.start_mission()

    def get_team_from_context(self, context):
        return context

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
