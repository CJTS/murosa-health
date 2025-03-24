import rclpy
import json
from rclpy.node import Node
from interfaces.srv import Message, Action
from std_msgs.msg import String
from murosa_plan_health.helper import FIPAMessage
from murosa_plan_health.FIPAPerformatives import FIPAPerformative

class Coordinator(Node):
    def __init__(self):
        super().__init__('Coordinator')
        self.robots = []
        self.arms = []
        self.nurses = []
        self.occ_robots = []
        self.occ_arms = []
        self.occ_nurses = []
        self.missions = []
        self.register_queue = []
        self.samples_queue = []

        # Coordinator server
        self._action_server = self.create_service(
            Message,
            'coordinator',
            self.execute_callback
        )
        self.jason_publisher = self.create_publisher(String, '/coordinator/jason/plan', 10)

        self.environment_client = self.create_client(
            Action, 'environment_server'
        )
        while not self.environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('environment service not available, waiting again...')

    def execute_callback(self, request, response):
        decoded_msg = FIPAMessage.decode(request.content)

        if decoded_msg.content == 'Register':
            if 'arm' in decoded_msg.sender:
                id = str(len(self.arms) + 1)
                response.response = id
                self.arms.append(decoded_msg.sender + id)
            elif 'robot' in decoded_msg.sender:
                id = str(len(self.robots) + 1)
                response.response = id
                self.robots.append(decoded_msg.sender + id)
            elif 'nurse' in decoded_msg.sender:
                id = str(len(self.nurses) + 1)
                response.response = id
                self.nurses.append(decoded_msg.sender + id)
            self.register_queue.append(decoded_msg.sender + id)
        elif decoded_msg.content == 'HasSample':
            self.samples_queue.append('Belief|nurse_has_sample(' + decoded_msg.sender + ')')

        self.get_logger().info(f'Received: Performative={decoded_msg.performative}, Sender={decoded_msg.sender}, Receiver={decoded_msg.receiver}, Content={decoded_msg.content}')
        return response

    def start_mission(self, nurse):
        team = self.get_team(nurse)

        if(team == None):
            print("No current team available")
            return

        self.get_logger().info(",".join(team))
        msg = String()
        msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', 'Jason', 'Start|' + ','.join(team)).encode()
        self.jason_publisher.publish(msg)
        return

    def get_agent_class(self, agent):
        return agent[:-1]

    def get_team(self, nurse):
        free_arms = list(set(self.arms) - set(self.occ_arms))
        free_robots = list(set(self.robots) - set(self.occ_robots))

        if len(free_arms) > 0 and len(free_robots) > 0:
            team = (free_arms[0], self.state['loc'][free_arms[0]], free_robots[0], self.state['loc'][free_robots[0]], nurse, self.state['loc'][nurse])
            self.occ_arms.append(free_arms[0])
            self.occ_robots.append(free_robots[0])
            self.occ_nurses.append(nurse)
            self.missions.append(team)
            return team

        return None

    def send_monitor_state_request(self, action):
        self.update_state_request = Action.Request()
        self.update_state_request.action = action
        return self.environment_client.call_async(self.update_state_request)

    def check_env(self):
        future = self.send_monitor_state_request(','.join(('monitor',)))
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.state = json.loads(response.observation)

        for key, value in self.state['sample'].items():
            if value and all(key not in team for team in self.missions):
                self.start_mission(key)

    def register_agents(self):
        if(len(self.register_queue) > 0):
            agent = self.register_queue.pop()
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Coordinator', 'Jason', 'Create|' + agent).encode()
            self.jason_publisher.publish(msg)

    def register_sample(self):
        if(len(self.samples_queue) > 0):
            sample = self.samples_queue.pop()
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.INFORM.value, 'Coordinator', 'Jason', sample).encode()
            self.jason_publisher.publish(msg)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            self.check_env()
            self.register_agents()
            self.register_sample()

def main():
    rclpy.init()
    coordinator = Coordinator()
    coordinator.get_logger().info('spin')
    try:
        # rclpy.spin(coordinator)
        coordinator.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    coordinator.get_logger().info('stop spin')
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
