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
        self.missions_with_error = []
        self.declare_parameter('replan', rclpy.Parameter.Type.BOOL)
        replan_param = self.get_parameter('replan').get_parameter_value().bool_value
        self.should_replan = replan_param

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

        self.planner_communication_sync_client = self.create_client(
            Action, 'planner_communication_sync_server'
        )
        while not self.planner_communication_sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('planner sync service not available, waiting again...')


    def execute_callback(self, request, response):
        decoded_msg = FIPAMessage.decode(request.content)

        if decoded_msg.performative == FIPAPerformative.REQUEST.value:
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
        elif decoded_msg.performative == FIPAPerformative.INFORM.value:
            if "ERROR" in decoded_msg.content:
                self.get_logger().info('Error found')
                mission = [mission for mission in self.missions if decoded_msg.sender in mission]
                self.missions_with_error.append(mission[0])

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

        self.update_planner_state(response.observation)

        for key, value in self.state['sample'].items():
            if value and all(key not in team for team in self.missions):
                self.start_mission(key)

    def send_update_state_request(self, state):
        self.update_state_request = Action.Request()
        self.update_state_request.action = state
        return self.planner_communication_sync_client.call_async(self.update_state_request)

    def update_planner_state(self, state):
        future = self.send_update_state_request('|'.join(('update_state', state)))
        rclpy.spin_until_future_complete(self, future)
        return future.result()

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

    def fix_plan(self, mission):
        self.get_logger().info(",".join(mission))

        arm = mission[0]
        robot = mission[2]
        nurse = mission[4]
        self.get_logger().info('Creating plan for: %s, %s, %s ' % (
            robot, nurse, arm
        ))
        future = self.send_need_plan_request(robot, nurse, arm)
        rclpy.spin_until_future_complete(self, future)
        plan_response = future.result()
        self.get_logger().info('Plan received for: %s, %s, %s ' % (
            robot, nurse, arm
        ))
        self.get_logger().info(plan_response.observation)
        self.current_plan = plan_response.observation.split('/')

        for action in self.current_plan:
            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.INFORM.value, 'Coordinator', 'Jason', 'Action|' + action).encode()
            self.jason_publisher.publish(msg)

    def fix_missions(self):
        if len(self.missions_with_error) > 0:
            mission = self.missions_with_error.pop()
            self.fix_plan(mission)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            self.check_env()
            self.register_agents()
            self.register_sample()
            self.fix_missions()

    def send_need_plan_request(self, robotid, nurseid, armid):
        self.action_request = Action.Request()
        self.action_request.action = '|'.join(
            ('need_plan', robotid, nurseid, armid))
        return self.planner_communication_sync_client.call_async(self.action_request)

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
