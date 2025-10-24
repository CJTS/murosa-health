import rclpy
import json
import random

from rclpy.node import Node
from std_msgs.msg import String, Bool

from interfaces.srv import Action, Message
from agents.helpers.helper import FIPAMessage
from agents.helpers.FIPAPerformatives import FIPAPerformative

class Environment(Node):
    def __init__(self):
        super().__init__('Environment')
        self.counter = 5000
        cleanOptions = [True, False]
        doorOptions = [False, True]
        nurse4Room = ['room4', 'icu']
        self.declare_parameter('problem_rate', rclpy.Parameter.Type.INTEGER)
        uncleaned_percentage = self.get_parameter('problem_rate').get_parameter_value().integer_value
        uncleaned1 = random.choices(cleanOptions, weights=(
            100 - uncleaned_percentage, uncleaned_percentage), k=1)
        uncleaned2 = random.choices(cleanOptions, weights=(
            100 - uncleaned_percentage, uncleaned_percentage), k=1)
        uncleaned3 = random.choices(cleanOptions, weights=(
            100 - uncleaned_percentage, uncleaned_percentage), k=1)
        uncleaned4 = random.choices(cleanOptions, weights=(
            100 - uncleaned_percentage, uncleaned_percentage), k=1)
        door1 = random.choices(doorOptions, weights=(
            100 - uncleaned_percentage, uncleaned_percentage), k=1)
        door2 = random.choices(doorOptions, weights=(
            100 - uncleaned_percentage, uncleaned_percentage), k=1)
        door3 = random.choices(doorOptions, weights=(
            100 - uncleaned_percentage, uncleaned_percentage), k=1)
        door4 = random.choices(doorOptions, weights=(
            100 - uncleaned_percentage, uncleaned_percentage), k=1)
        icuRoom = random.choices(nurse4Room, weights=(
            100 - uncleaned_percentage, uncleaned_percentage), k=1)

        self.state = {
            'loc': {
                'nurse1': 'nr',
                'nurse2': 'nr',
                'nurse3': 'nr',
                'nurse4': 'nr',
                'uvd1': 'ds',
                'spot1': 'ds',
                'uvd2': 'ds',
                'spot2': 'ds',
                'collector1': 'ds',
                'collector2': 'ds',
                'arm1': 'lab'
            },
            'pos': {
                'nurse1': (185, 125),
                'nurse2': (185, 125),
                'nurse3': (185, 125),
                'nurse4': (185, 125),
                'uvd1': (235, 50),
                'spot1': (235, 50),
                'uvd2': (235, 50),
                'spot2': (235, 50),
                'collector1': (235, 50),
                'collector2': (235, 50),
                'arm1': (185, 50)
            },
            'doors': {
                'room1': door1[0],
                'room2': door2[0],
                'room3': door3[0],
                'room4': door4[0],
                'icu': door4[0]
            },
            'cleaned': {
                'room1': uncleaned1[0],
                'room2': uncleaned2[0],
                'room3': uncleaned3[0],
                'room4': uncleaned4[0],
                'icu': uncleaned4[0]
            },
            'samples': {
                'room1': False,
                'room2': False,
                'room3': False,
                'room4': False,
                'room5': False,
                'room6': False,
                'icu': False
            },
            'disinfected': {
                'room1': True,
                'room2': True,
                'room3': True,
                'room4': True,
                'room5': True,
                'room6': True,
                'icu': True
            }
        }
        self.start_server()

    def start_server(self):
        self.get_logger().info('Starting Environment server')
        self.environment_server = self.create_service(
            Action, 'environment_server', self.receive_message
        )
        self.end_simulation_subscription = self.create_subscription(
            Bool, '/coordinator/shutdown_signal', self.end_simulation_callback, 10
        )
        self.publisher = self.create_publisher(String, '/env/front/state', 10)
        self.cli = self.create_client(Message, 'coordinator')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.get_logger().info('Environment server started')

    def end_simulation_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento do coordenador, finalizando...")
            raise SystemExit

    def receive_message(self, request, response):
        actionTuple = tuple(request.action.split(','))
        response.observation = 'success'

        if actionTuple[0] == 'a_open_door':
            self.state['doors'][actionTuple[2]] = True
            response.observation = 'success'
        elif actionTuple[0] == 'a_navto':
            self.state['loc'][actionTuple[1]] = actionTuple[2]
            response.observation = 'success'
        elif actionTuple[0] == 'monitor':
            response.observation = json.dumps(self.state)
        elif actionTuple[0] == 'a_infected_room':
            room = self.state['loc'][actionTuple[1]]
            self.state['disinfected'][room] = False
            self.get_logger().info(f"Room {room} marked as infected")
        elif actionTuple[0] == 'a_patrol_room' and not self.state['cleaned'][actionTuple[2]]:
            response.observation = 'dirty room'
        elif actionTuple[0] == 'a_clean_room':
            self.state['cleaned'][actionTuple[2]] = True
            response.observation = 'success'
        elif actionTuple[0] == 'a_disinfect_room':
            if self.state['cleaned'][actionTuple[2]]:
                self.state['disinfected'][actionTuple[2]] = True
                response.observation = 'success'
        elif actionTuple[0] == 'what_room':
            response.observation = self.state['loc'][actionTuple[1]]
        elif actionTuple[0] == 'a_collect_sample':
            self.state['disinfected'][actionTuple[2]] = False
        elif actionTuple[0] == 'move':
            self.state['pos'][actionTuple[1]] = (
                self.state['pos'][actionTuple[1]][0] + float(actionTuple[2]),
                self.state['pos'][actionTuple[1]][1] + float(actionTuple[3])
            )
            response.observation = ','.join([str(self.state['pos'][actionTuple[1]][0]), str(self.state['pos'][actionTuple[1]][1])])

        return response

    def sample_initial_trigger(self, room):
        message = FIPAMessage(FIPAPerformative.INFORM.value, 'Env', 'Coordinator', 'InitialTrigger|Sample,' + room).encode()
        ros_msg = Message.Request()
        ros_msg.content = message
        return self.cli.call_async(ros_msg)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            self.counter = self.counter + 1
            if self.counter == 10000:
                rooms_list = ["room1", "room2", "room3", "room4", "room5", "room6", "icu"]
                random_room = random.choice(rooms_list)
                self.state['samples'][random_room] = True
                future = self.sample_initial_trigger(random_room)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                self.get_logger().info('Initial Trigger:  %s' % (random_room))

            msg = String()
            msg.data = FIPAMessage(FIPAPerformative.REQUEST.value, 'Env', 'Front', json.dumps(self.state)).encode()
            self.publisher.publish(msg)

def main():
    rclpy.init()
    environment = Environment()
    try:
        environment.run()
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    environment.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()