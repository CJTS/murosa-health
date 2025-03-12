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
            'loc': {'nurse1': 'Room1', 'robot1': 'Room2', 'arm1': 'Room3'},
            'doors': {'Room1': True, 'Room2': True, 'Room3': closed_door[0]},
            'sample': {'nurse1': True, 'robot1': False, 'arm1': False}
        }

    def start_server(self):
        self.get_logger().info('Starting Environment server')
        self.environment_server = self.create_service(
            Action, 'environment_server', self.receive_message
        )
        self.get_logger().info('Environment server started')

    def receive_message(self, request, response):
        self.get_logger().info('Receiving message: %s' % request.action)
        actionTuple = tuple(request.action.split(','))
        self.get_logger().info('Action: %s' % actionTuple[0])
        if actionTuple[0] == 'a_navto' and not self.state['doors'][actionTuple[2]]:
            response.observation = 'door closed'
            self.get_logger().info('Responding')
            return response
        elif actionTuple[0] == 'a_open_door':
            self.state['doors'][actionTuple[2]] = True
            self.get_logger().info('Responding')
            response.observation = 'success'
            return response
        elif actionTuple[0] == 'monitor':
            response.observation = json.dumps(self.state)
            return response
        self.get_logger().info('Responding')
        response.observation = 'success'
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
