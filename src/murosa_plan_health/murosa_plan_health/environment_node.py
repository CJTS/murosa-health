from threading import Thread
import sys
import rclpy
from rclpy.node import Node
from interfaces.srv import Action
import json
import random

class Environment(Node):

    def __init__(self):
        super().__init__('Environment')
        self.client_futures = []
        self.door = False
        numberList = [True, False]
        self.declare_parameter('problem_rate', rclpy.Parameter.Type.INTEGER)
        closed_door_percentage = self.get_parameter('problem_rate').get_parameter_value().integer_value
        print(closed_door_percentage)
        closed_door = random.choices(numberList, weights=(
            100 - closed_door_percentage, closed_door_percentage), k=1)

        self.state = {
            'loc': {'nurse1': 'room1','nurse2': 'room3', 'robot1': 'room2', 'robot2': 'room2', 'arm1': 'room4', 'arm2': 'room4'},
            'doors': {'room1': closed_door[0], 'room2': True, 'room3': closed_door[0], 'room4': True},
            'sample': {'nurse1': False, 'nurse2': False, 'robot1': False, 'robot2': False, 'arm1': False, 'arm2': False}
        }

    def start_server(self):
        self.get_logger().info('Starting Environment server')
        self.environment_server = self.create_service(
            Action, 'environment_server', self.receive_message
        )
        self.get_logger().info('Environment server started')

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
        elif actionTuple[0] == 'a_create_sample':
            self.state['sample'][actionTuple[1]] = True

        return response

    def run(self):
        while rclpy.ok():
            i = 0


def main():
    rclpy.init()
    environment = Environment()
    environment.start_server()
    spin_thread = Thread(target=rclpy.spin, args=(environment,))
    spin_thread.start()
    environment.run()
    environment.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
