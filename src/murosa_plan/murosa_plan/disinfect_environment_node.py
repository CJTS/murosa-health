from threading import Thread
import sys
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
        #self.door = False
        #numberList = [True, False]
        self.declare_parameter('problem_rate', rclpy.Parameter.Type.INTEGER)
        #closed_door_percentage = self.get_parameter('problem_rate').get_parameter_value().integer_value
        #closed_door = random.choices(numberList, weights=(
            # 100 - closed_door_percentage, closed_door_percentage), k=1)
        
        self.state = {
            'loc': { 'nurse_disinfected1': 'room1','nurse_disinfected2': 'room2', 'nurse_disinfected3': 'room3','uvdrobot1': 'room4', 'spotrobot1': 'room4'},
            'doors': { 'room1': True, 'room2': True, 'room3': True, 'room4': True },
            'cleaned': { 'room1' : True,'room2' : True, 'room3' : True },
            'disinfected': {'room1': False,'room2': True, 'room3': True}
            }
        self.start_server()

        
    def start_server(self):
        self.get_logger().info('Starting Environment server')
        self.environment_server = self.create_service(
            Action, 'environment_server', self.receive_message
        )

        # Subscriber para indicar fim da execução
        self.end_subscription = self.create_subscription(
            Bool, '/jason/shutdown_signal', self.shutdown_callback, 10
        )
        self.end_simulation_subscription = self.create_subscription(
            Bool, '/coordinator/shutdown_signal', self.end_simulation_callback, 10
        )

        self.get_logger().info('Environment server started')

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento, finalizando...")
            raise SystemExit
        
    def end_simulation_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento do coordenador, finalizando...")
            raise SystemExit

    def receive_message(self, request, response):
        actionTuple = tuple(request.action.split(','))
        response.observation = 'success'

        if actionTuple[0] == 'a_navto' and not self.state['doors'][actionTuple[2]]:
            response.observation = 'door closed'
        elif actionTuple[0] == 'a_open_door':
            self.state['doors'][actionTuple[2]] = True
            response.observation = 'success'
        elif actionTuple[0] == 'monitor':
            response.observation = json.dumps(self.state)
        elif actionTuple[0] == 'a_infected_room':
            self.state['disinfected'][actionTuple[1]] = False
        elif actionTuple[0] == 'a_patrol_room' and not self.state['cleaned'][actionTuple[2]]:
            response.observation = 'dirty room'
        elif actionTuple[0] == 'a_disinfect_room' and not self.state['cleaned'][actionTuple[2]]:
            response.observation = 'dirty room'

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