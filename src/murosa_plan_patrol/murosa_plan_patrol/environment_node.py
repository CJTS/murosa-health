from threading import Thread
import sys
import rclpy
from rclpy.node import Node
from interfaces.srv import Action
import json
import random
from std_msgs.msg import String, Bool

class Environment(Node):
    def __init__(self):
        super().__init__('Environment')
        self.declare_parameter('problem_rate', rclpy.Parameter.Type.INTEGER)
        self.start_server()
        self.state = {
            'wps': {'wp1': False, 'wp2': False, 'wp3': False, 'wp4': False},
        }

    def start_server(self):
        self.get_logger().info('Starting Environment server')
        self.environment_server = self.create_service(
            Action, 'environment_server', self.receive_message
        )
        self.end_subscription = self.create_subscription(
            Bool, '/shutdown_signal', self.shutdown_callback, 10
        )
        self.get_logger().info('Environment server started')

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recebido sinal de desligamento, finalizando...")
            raise SystemExit

    def receive_message(self, request, response):
        actionTuple = tuple(request.action.split(','))
        if actionTuple[0] == 'monitor':
            response.observation = json.dumps(self.state)
            return response
        self.get_logger().info('Responding')
        response.observation = 'success'
        return response

def main():
    rclpy.init()
    environment = Environment()
    environment.get_logger().info('spin')
    try:
        rclpy.spin(environment)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    environment.get_logger().info('stop spin')
    environment.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
