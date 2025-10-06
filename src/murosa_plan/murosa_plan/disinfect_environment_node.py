import rclpy
import json
import random
from rclpy.node import Node
from interfaces.srv import Action
from std_msgs.msg import Bool

class Environment(Node):

    def __init__(self):
        super().__init__('Environment')
        self.client_futures = []
        self.door = False
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
                'nurse_disinfected1': 'room1',
                'nurse_disinfected2': icuRoom[0], 
                'nurse_disinfected3': 'room3',
                'nurse_disinfected4': icuRoom[0],
                'uvdrobot1': 'room4', 
                'spotrobot1': 'room4',
                'uvdrobot2': 'room4', 
                'spotrobot2': 'room4'
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
            'disinfected': {
                'room1': True,
                'room2': True,
                'room3': True,
                'room4': True,
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
                
        return response

    def run(self):
        while rclpy.ok():
            i = 0

def main():
    rclpy.init()
    environment = Environment()
    try:
        rclpy.spin(environment)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    environment.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()